/*! ***************************************************************************
 *
 * \brief     EVKD5 main application
 * \file      image.h
 * \author    Hugo Arends - HAN Embedded Vision and Machine Learning
 * \author
 * \date      April 2024
 *
 * \copyright 2024 HAN University of Applied Sciences. All Rights Reserved.
 *            \n\n
 *            Permission is hereby granted, free of charge, to any person
 *            obtaining a copy of this software and associated documentation
 *            files (the "Software"), to deal in the Software without
 *            restriction, including without limitation the rights to use,
 *            copy, modify, merge, publish, distribute, sublicense, and/or sell
 *            copies of the Software, and to permit persons to whom the
 *            Software is furnished to do so, subject to the following
 *            conditions:
 *            \n\n
 *            The above copyright notice and this permission notice shall be
 *            included in all copies or substantial portions of the Software.
 *            \n\n
 *            THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 *            EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 *            OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 *            NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 *            HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 *            WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *            FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 *            OTHER DEALINGS IN THE SOFTWARE.
 *
 *****************************************************************************/
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "gpio_input.h"
#include "gpio_output.h"

#include "uvc_camera.h"

#include "fsl_clock.h"
#include "fsl_smartdma.h"
#include "fsl_inputmux.h"
#include "fsl_inputmux_connections.h"
#include "fsl_ov7670.h"

#include "operators.h"

// -----------------------------------------------------------------------------
// Local type definitions
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Local function prototypes
// -----------------------------------------------------------------------------
void systemInit(void);
void exampleWebcamBgr888(void);
void exampleWebcamUint8(void);
void exampleWebcamBgr888TestPattern(void);
void exampleWebcamUint8TestPattern(void);
void exampleThreshold(void);
void exampleRotate(void);
void exampleTemplate(void);
void exampleConvolve(void);
void exampleMean(void);
void exampleContrast(void);
void exampleScale(void);
void exampleScaleFast(void);
void exampleClearUint8(void);
void exampleClearUint8Cm33(void);
void exampleConvolveFast(void);
void exampleMeanFast(void);
void exampleFinalAssignment(void);
void exampleThreshold2Means(void);
void exampleThresholdOtsu(void);
void exampleSobelFast(void);
void exampleRemoveBorderBlobs(void);
void exampleFillHoles(void);
void exampleLabel(void);
void examplePerimeter(void);

#if (USB_IMAGE_TYPE_UYVY == 1)
void exampleWebcamUyvy(void);
#endif

// -----------------------------------------------------------------------------
// Local variables
// -----------------------------------------------------------------------------

// Note: The heap (malloc()) is used for allocating image data
//
// uint8_pixel_t : 160 * 120 * 1 = 19200 bytes per image
// int16_pixel_t : 160 * 120 * 2 = 38400 bytes per image
// int32_pixel_t : 160 * 120 * 4 = 76800 bytes per image
// uyvy_pixel_t  : 160 * 120 * 2 = 38400 bytes per image
// bgr888_pixel_t: 160 * 120 * 3 = 57600 bytes per image
// float_pixel_t : 160 * 120 * 4 = 76800 bytes per image
//
// The heap is configured to 345888 (0x54720) bytes
// Image data for 345888 / (160 * 120) = 18.015 uint8_pixel_t images can be
// allocated. However, the images called 'cam' and 'usb' are mandatory and are
// already taking 5 (2+3) of these.

// Globally shared images. Allocation of these images is taken care of in
// systemInit(). Not static, so these can be referenced in other source files.
image_t *cam = NULL;
image_t *usb = NULL;

// Variables for timing
static volatile uint32_t ms = 0;
static volatile uint32_t ms1 = 0;
static volatile uint32_t ms2 = 0;

// Variables for SmartDMA
static volatile uint8_t smartdma_stack[64];
static volatile uint32_t smartdma_camera_image_complete = 0;
static smartdma_param_t smartdma_param;

// Variable for USB
// Not static, because referenced in uvc_camera.c
volatile uint32_t image_available_for_usb = 0;

// Camera configuration settings
static ov7670_resource_t ov7670_resource =
{
    .xclock = kOV7670_InputClock12MHZ,
    .i2cReceiveFunc = BOARD_Camera_I2C_ReceiveSCCB,
    .i2cSendFunc = BOARD_Camera_I2C_SendSCCB,
};

static camera_device_handle_t camera_device_handle =
{
    .ops = &ov7670_ops,
    .resource = &ov7670_resource,
};

static const camera_config_t camera_config =
{
    // uyuv_pixel_t takes 2 bytes per pixel
    .bytesPerPixel = 2,
    // Supported frame rates are: 14, 15, 25 and 30 fps
    .framePerSec = 30,
    // Although the define says YUYV, the OV7670 camera module is configured as
    // follows after initialization (also see OV7670 datasheet):
    // TSLB:  0x08 = 0b0000_1000
    // COM13: 0x88 = 0b1000_1000
    // TSLB[3], COM13[0] = 10 => UYVY
    //
    // This means pixels are stored in memory as follows (U is LSB, Y is MSB):
    //  31                                0
    // |YYYYYYYY VVVVVVVV|YYYYYYYY UUUUUUUU|
    // |     pixel 1     |     pixel 0     |
    .pixelFormat = kVIDEO_PixelFormatYUYV,
    // HSYNC/HREF, VSYNC, and PIXCLK signals are used, which is called Gated
    // Clock interface
    .interface = kCAMERA_InterfaceGatedClock,
    // QQVGA (160x120) is the supported resolution due to memory constraints
    .resolution = kVIDEO_ResolutionQQVGA,
};

// -----------------------------------------------------------------------------
// Function implementation
// -----------------------------------------------------------------------------

// Callback function that is called in the IRQ handler of the DMA controller. It
// is called when a new image is available in the cam image. Poll this flag in
// the main loop.
static void SmartDMA_camera_callback(void *param)
{
    smartdma_camera_image_complete = 1;
}

int main(void)
{
    // -------------------------------------------------------------------------
    // Initialize the system
    // -------------------------------------------------------------------------
    systemInit();

    PRINTF("Application start\r\n");

    // -------------------------------------------------------------------------
    // Start webcam mode if SW2 is pressed during startup
    // -------------------------------------------------------------------------
    if((GPIO0->PDIR & (1<<23)) == 0)
    {
        PRINTF("SW2 pressed - starting bgr888 webcam mode\r\n");
        exampleWebcamBgr888();
    }

    // -------------------------------------------------------------------------
    // Select one example
    // -------------------------------------------------------------------------

//    exampleWebcamBgr888();
//    exampleWebcamUint8();
//    exampleWebcamBgr888TestPattern();
//    exampleWebcamUint8TestPattern();
//    exampleThreshold();
//    exampleRotate();
//    exampleTemplate();
//    exampleContrast();
//    exampleScale();
//    exampleScaleFast();
//    exampleClearUint8();
//    exampleClearUint8Cm33();
//    exampleConvolve();
//    exampleMean();
//    exampleConvolveFast();
//    exampleMeanFast();
    exampleFinalAssignment();
//    exampleThreshold2Means();
//    exampleThresholdOtsu();
//    exampleSobelFast();
//    exampleRemoveBorderBlobs();
//	  exampleFillHoles();
//    exampleLabel();
//    examplePerimeter();

    // -------------------------------------------------------------------------
    // Should never reach this
    // -------------------------------------------------------------------------
    PRINTF("Error. Should never reach this when one of the examples is " \
        "enabled\r\n");

    while(1U)
    {
        // Intentionally left blank
    }
}

void systemInit(void)
{
    status_t status;

    // -------------------------------------------------------------------------
    // Board
    BOARD_InitBootPins();
    BOARD_PowerMode_OD();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();
    CLOCK_SetupExtClocking(BOARD_XTAL0_CLK_HZ);

    PRINTF("\r\n\r\n");
    PRINTF("EVDK5.0 - %s %s\r\n", __DATE__, __TIME__);
    PRINTF("Application init\r\n");

    // -------------------------------------------------------------------------
    // Image memory allocation for static images required by camera and USB.
    // Allocation must be done before DMA initialization!
    cam = newUyvyImage(EVDK5_WIDTH, EVDK5_HEIGHT);

#if (USB_IMAGE_TYPE_BGR888 == 1)
    usb = newBgr888Image(EVDK5_WIDTH, EVDK5_HEIGHT);
#endif

#if (USB_IMAGE_TYPE_UYVY == 1)
    usb = newUyvyImage(EVDK5_WIDTH, EVDK5_HEIGHT);
#endif

    if(usb == NULL)
    {
        PRINTF("Error. Could not allocate image memory\r\n");
        while(1)
        {}
    }

    // -------------------------------------------------------------------------
    // SysTick
    status = SysTick_Config(SystemCoreClock/1000);

    if(status != 0)
    {
        PRINTF("Error. SysTick not started\r\n");
        while(1)
        {}
    }

    // Lowest priority for the SysTick timer
    NVIC_SetPriority(SysTick_IRQn, 7);

    // -------------------------------------------------------------------------
    // GPIOs
    gpio_output_init();
    gpio_input_init();

    // -------------------------------------------------------------------------
    // OV7670 camera
    GPIO1->PCOR = (1<<19); // CAMERA_RST: 0 normal mode; 1 reset mode

    // 10ms settling time for the camera module
    ms=0;
    while(ms < 10)
    {}

    GPIO1->PSOR = (1<<19); // CAMERA_RST: 0 normal mode; 1 reset mode

    // Set alternative 7 (SmartDMA_PIOn) for all data pins
    PORT1->PCR[4] = PORT_PCR_MUX(7) | PORT_PCR_IBE(1);
    PORT1->PCR[5] = PORT_PCR_MUX(7) | PORT_PCR_IBE(1);
    PORT1->PCR[6] = PORT_PCR_MUX(7) | PORT_PCR_IBE(1);
    PORT1->PCR[7] = PORT_PCR_MUX(7) | PORT_PCR_IBE(1);
    PORT3->PCR[4] = PORT_PCR_MUX(7) | PORT_PCR_IBE(1);
    PORT3->PCR[5] = PORT_PCR_MUX(7) | PORT_PCR_IBE(1);
    PORT1->PCR[10] = PORT_PCR_MUX(7) | PORT_PCR_IBE(1);
    PORT1->PCR[11] = PORT_PCR_MUX(7) | PORT_PCR_IBE(1);

    // Enable and set clocks for LPFLEXCOMM7 and LPI2C7
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM7);
    CLOCK_EnableClock(kCLOCK_LPFlexComm7);
    CLOCK_EnableClock(kCLOCK_LPI2c7);
    CLOCK_SetClkDiv(kCLOCK_DivFlexcom7Clk, 1u);

    // Initialize LPI2C7
    BOARD_Camera_I2C_Init();

    status = CAMERA_DEVICE_Init(&camera_device_handle, &camera_config);

    if(status != kStatus_Success)
    {
        PRINTF("Error. Camera not initialized\r\n");
        while(1)
        {}
    }

    // -------------------------------------------------------------------------
    // SmartDMA
    // SmartDMA firmware is copied from FLASH to SRAMX
    SMARTDMA_Init(SMARTDMA_CAMERA_MEM_ADDR, s_smartdmaCameraFirmware,
        SMARTDMA_CAMERA_FIRMWARE_SIZE);

    // Set the callback function. This function will be called when an entire
    // frame from the camera is available.
    SMARTDMA_InstallCallback(SmartDMA_camera_callback, NULL);

    // Enable SmartDMA interrupts with highest priority
    NVIC_SetPriority(SMARTDMA_IRQn, 0);
    NVIC_EnableIRQ(SMARTDMA_IRQn);

    // SmartDMA core needs its own stack. The file fsl_smartdma_mcxn.h
    // describes that is shall be at least 64 bytes.
    smartdma_param.cameraParam.smartdma_stack = (uint32_t *)smartdma_stack;
    // Configure pointer for storing camera data
    smartdma_param.cameraParam.p_buffer = (uint32_t *)(cam->data);
    // Boot the SMARTDMA to run program.
    SMARTDMA_Boot(kSMARTDMA_FlexIO_CameraWholeFrame, &smartdma_param, 0x2);

    // -------------------------------------------------------------------------
    // USB
    USB_DeviceApplicationInit();

#if (USB_IMAGE_TYPE_UYVY == 1)
    // In UYVY webcam mode, always start the example here. This keeps main nice
    // and clean
    exampleWebcamUyvy();
#endif
}

// System tick timer IRQ handler. Is called every millisecond.
void SysTick_Handler(void)
{
    ms++;
}

// -----------------------------------------------------------------------------
// BGR888 examples
// -----------------------------------------------------------------------------

#if (USB_IMAGE_TYPE_BGR888 == 1)
void exampleWebcamBgr888(void)
{
    PRINTF("%s\r\n", __func__);

    while(1U)
    {
        // ---------------------------------------------------------------------
        // Wait for camera image complete
        // ---------------------------------------------------------------------
        while(smartdma_camera_image_complete == 0)
        {}

        smartdma_camera_image_complete = 0;

        // Copy timestamp
        ms1 = ms;

        // ---------------------------------------------------------------------
        // Image processing pipeline
        // ---------------------------------------------------------------------
        // Convert uyvy_pixel_t camera image to bgr888_pixel_t image for USB
        convertUyvyToBgr888(cam, usb);

#if 0
        // ---------------------------------------------------------------------
        // Optionally wait while frame is being handled by USB.
        // This will wait forever if there is no USB connection!
        // ---------------------------------------------------------------------
        while(image_available_for_usb == 1)
        {}
#endif

        // ---------------------------------------------------------------------
        // Set flag for USB interface that a new frame is available
        // ---------------------------------------------------------------------
        image_available_for_usb = 1;

        // Copy timestamp
        ms2 = ms;

        // Print debug info
        PRINTF("%d | delta: %d ms\r\n", ms1, ms2-ms1);
    }
}

void exampleWebcamUint8(void)
{
    PRINTF("%s\r\n", __func__);

    // -------------------------------------------------------------------------
    // Local image memory allocation
    // -------------------------------------------------------------------------
    image_t *src = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT);
    image_t *dst = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT);

    if(dst == NULL)
    {
        PRINTF("Could not allocate image memory\r\n");
        while(1)
        {}
    }

    while(1U)
    {
        // ---------------------------------------------------------------------
        // Wait for camera image complete
        // ---------------------------------------------------------------------
        while(smartdma_camera_image_complete == 0)
        {}

        smartdma_camera_image_complete = 0;

        // Copy timestamp
        ms1 = ms;

        // ---------------------------------------------------------------------
        // Image processing pipeline
        // ---------------------------------------------------------------------
        // Convert uyvy_pixel_t camera image to uint8_pixel_t image
        convertUyvyToUint8(cam, src);

        // Examples, select one!
        copyUint8Image(src, dst);
//        scale(src, dst);
//        brightness(src, dst, 100);
//        contrast(src, dst, 2.0f);

        // Convert uint8_pixel_t image to bgr888_pixel_t image for USB
        convertUint8ToBgr888(dst, usb);

        // ---------------------------------------------------------------------
        // Set flag for USB interface that a new frame is available
        // ---------------------------------------------------------------------
        image_available_for_usb = 1;

        // Copy timestamp
        ms2 = ms;

        // Print debug info
        PRINTF("%d | delta: %d ms\r\n", ms1, ms2-ms1);
    }
}

void exampleWebcamBgr888TestPattern(void)
{
    PRINTF("%s\r\n", __func__);

    // -------------------------------------------------------------------------
    // Local image memory allocation
    // -------------------------------------------------------------------------
    image_t *bgr888 = newBgr888Image(EVDK5_WIDTH, EVDK5_HEIGHT);

    const bgr888_pixel_t colorLut[] =
    {
        {.b=0x00, .g=0x00, .r=0xFF}, // Red
        {.b=0x00, .g=0xFF, .r=0x00}, // Green
        {.b=0xFF, .g=0x00, .r=0x00}, // Blue
        {.b=0x00, .g=0xFF, .r=0xFF}, // Yellow
        {.b=0xFF, .g=0x00, .r=0xFF}, // Magenta
        {.b=0xFF, .g=0xFF, .r=0x00}, // Cyan
        {.b=0xFF, .g=0xFF, .r=0xFF}, // White
        {.b=0x00, .g=0x00, .r=0x00}, // Black
    };

    const uint32_t numberOfColors = sizeof(colorLut) / sizeof(bgr888_pixel_t);
    bgr888_pixel_t color = {0x00, 00, 00};
    uint32_t cnt = 0;

    while(1U)
    {
        // ---------------------------------------------------------------------
        // Wait for camera image complete
        // ---------------------------------------------------------------------
        while(smartdma_camera_image_complete == 0)
        {}

        smartdma_camera_image_complete = 0;

        // ---------------------------------------------------------------------
        // Image processing pipeline
        // ---------------------------------------------------------------------
        // Change color every second
        if(ms1 < ms)
        {
            ms1 = ms + 1000;

            // Set color
            color = colorLut[cnt];

            // Calculate next index
            cnt = (cnt == (numberOfColors-1)) ? 0 : cnt + 1;
        }

        // \todo Copy-and-paste week 1 code here



        // Convert bgr888_pixel_t image to USB
        copyBgr888Image(bgr888, usb);

        // ---------------------------------------------------------------------
        // Set flag for USB interface that a new frame is available
        // ---------------------------------------------------------------------
        image_available_for_usb = 1;
    }
}

void exampleWebcamUint8TestPattern(void)
{
    PRINTF("%s\r\n", __func__);

    // -------------------------------------------------------------------------
    // Local image memory allocation
    // -------------------------------------------------------------------------
    image_t *img0 = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT);

    while(1U)
    {
        // ---------------------------------------------------------------------
        // Wait for camera image complete
        // ---------------------------------------------------------------------
        while(smartdma_camera_image_complete == 0)
        {}

        smartdma_camera_image_complete = 0;

        // ---------------------------------------------------------------------
        // Image processing pipeline
        // ---------------------------------------------------------------------
        // Set test pattern: pixel value is row number
        for(int r=0; r < img0->rows; r++)
        {
            memset(img0->data + (r * img0->cols), r, img0->cols);
        }

        // Convert uint8_pixel_t image to bgr888_pixel_t image for USB
        convertToBgr888(img0, usb);

        // ---------------------------------------------------------------------
        // Set flag for USB interface that a new frame is available
        // ---------------------------------------------------------------------
        image_available_for_usb = 1;
    }
}
#endif

// -----------------------------------------------------------------------------
// UYVY examples
// -----------------------------------------------------------------------------

#if (USB_IMAGE_TYPE_UYVY == 1)
void exampleWebcamUyvy(void)
{
    PRINTF("%s\r\n", __func__);

    while(1U)
    {
        // Wait for camera image complete
        while(smartdma_camera_image_complete == 0)
        {}

        // Reset the flag
        smartdma_camera_image_complete = 0;

        // Copy the image. To speed thing up even more, the usb and cam images
        // can be made to point to the same image data buffer.
        copyUyvyImage(cam,  usb);

        // Set flag for USB interface that a new frame is available
        image_available_for_usb = 1;
    }
}
#endif

// -----------------------------------------------------------------------------
// Other examples
// -----------------------------------------------------------------------------

void exampleThreshold(void)
{
    PRINTF("%s\r\n", __func__);

    // -------------------------------------------------------------------------
    // Local image memory allocation
    // -------------------------------------------------------------------------
    image_t *src = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT);
    image_t *dst = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT);

    if(dst == NULL)
    {
        PRINTF("Could not allocate image memory\r\n");
        while(1)
        {}
    }

    while(1U)
    {
        // ---------------------------------------------------------------------
        // Wait for camera image complete
        // ---------------------------------------------------------------------
        while(smartdma_camera_image_complete == 0)
        {}

        smartdma_camera_image_complete = 0;

        // Copy timestamp
        ms1 = ms;

        // ---------------------------------------------------------------------
        // Image processing pipeline
        // ---------------------------------------------------------------------
        // Convert uyvy_pixel_t camera image to uint8_pixel_t image
        convertToUint8(cam, src);

        threshold(src, dst, 0, 64);

        // Convert uint8_pixel_t image to bgr888_pixel_t image for USB
        convertToBgr888(dst, usb);

        // Copy timestamp
        ms2 = ms;

        // ---------------------------------------------------------------------
        // Set flag for USB interface that a new frame is available
        // ---------------------------------------------------------------------
        image_available_for_usb = 1;

        // Print debug info
        PRINTF("%d | delta: %d ms\r\n", ms1, ms2-ms1);
    }
}


void exampleRotate(void)
{
    PRINTF("%s\r\n", __func__);

    // Update SysTick to increase precision to 0.01ms (=10us)
    uint32_t status = SysTick_Config(SystemCoreClock/100000);

    if(status != 0)
    {
        PRINTF("SysTick update failed\r\n");
        while(1)
        {}
    }

    PRINTF("SysTick updated successfully\r\n");

    // -------------------------------------------------------------------------
    // Local image memory allocation
    // -------------------------------------------------------------------------
    image_t *src = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT);
    image_t *dst = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT);

    if(dst == NULL)
    {
        PRINTF("Could not allocate image memory\r\n");
        while(1)
        {}
    }

    while(1U)
    {
        // ---------------------------------------------------------------------
        // Wait for camera image complete
        // ---------------------------------------------------------------------
        while(smartdma_camera_image_complete == 0)
        {}

        smartdma_camera_image_complete = 0;

        // ---------------------------------------------------------------------
        // Image processing pipeline
        // ---------------------------------------------------------------------
        convertToUint8(cam, src);

        // Prepare
        clearUint8Image(dst);

        ms1 = ms;
        rotate(src, dst, 3.1415f, (point_t){src->cols / 2, src->rows / 2});
        ms2 = ms;
        PRINTF("%06d us", (ms2-ms1)*10);

        // Prepare
        copyUint8Image(src, dst);

        ms1 = ms;
        rotate180_c(dst);
        ms2 = ms;
        PRINTF(" | %03d us", (ms2-ms1)*10);

        // Prepare
        copyUint8Image(src, dst);

        ms1 = ms;
        rotate180_arm(dst);
        ms2 = ms;
        PRINTF(" | %03d us", (ms2-ms1)*10);

        // Prepare
        copyUint8Image(src, dst);

        ms1 = ms;
        rotate180_cm33(dst);
        ms2 = ms;
        PRINTF(" | %03d us", (ms2-ms1)*10);

        PRINTF("\r\n");

        convertToBgr888(dst, usb);

        // ---------------------------------------------------------------------
        // Set flag for USB interface that a new frame is available
        // ---------------------------------------------------------------------
        image_available_for_usb = 1;
    }
}

void exampleTemplate(void)
{
    PRINTF("%s\r\n", __func__);

    // Update SysTick to have better precision
    SysTick_Config(SystemCoreClock/100000);

    // -------------------------------------------------------------------------
    // Local image memory allocation
    // -------------------------------------------------------------------------
    image_t *src = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT);
    image_t *dst = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT);

    if(dst == NULL)
    {
        PRINTF("Could not allocate image memory\r\n");
        while(1)
        {}
    }

    while(1U)
    {
        // ---------------------------------------------------------------------
        // Wait for camera image complete
        // ---------------------------------------------------------------------
        while(smartdma_camera_image_complete == 0)
        {}

        smartdma_camera_image_complete = 0;

        // ---------------------------------------------------------------------
        // Image processing pipeline
        // ---------------------------------------------------------------------
        // Convert uyvy_pixel_t camera image to uint8_pixel_t image
        convertToUint8(cam, src);

        // Copy timestamp
        ms1 = ms;

        // \todo
        // Use this as a playground for testing image processing functions. As
        // an example, the following function scales the image for better
        // visualization.
        scale(src, dst);

        // Copy timestamp
        ms2 = ms;

        // Convert uint8_pixel_t image to bgr888_pixel_t image for USB
        convertToBgr888(dst, usb);

        // ---------------------------------------------------------------------
        // Set flag for USB interface that a new frame is available
        // ---------------------------------------------------------------------
        image_available_for_usb = 1;

        // Print debug info
        PRINTF("%d | delta: %04d us\r\n", ms1, (ms2-ms1)*10);
    }
}

void exampleContrast(void)
{
    PRINTF("%s\r\n", __func__);

    // -------------------------------------------------------------------------
    // Local image memory allocation
    // -------------------------------------------------------------------------
    image_t *src = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT);
    image_t *dst = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT);

    if(dst == NULL)
    {
        PRINTF("Could not allocate image memory\r\n");
        while(1)
        {}
    }

    float contrastValue = 1.0f;
    bool increasing = true;

    while(1U)
    {
        // ---------------------------------------------------------------------
        // Wait for camera image complete
        // ---------------------------------------------------------------------
        while(smartdma_camera_image_complete == 0)
        {}

        smartdma_camera_image_complete = 0;

        // Copy timestamp
        ms1 = ms;

        // ---------------------------------------------------------------------
        // Image processing pipeline
        // ---------------------------------------------------------------------
        // Convert uyvy_pixel_t camera image to uint8_pixel_t image
        convertUyvyToUint8(cam, src);

        // Apply contrast with varying contrast value
        contrast(src, dst, contrastValue);

        // Gradually change contrast value between 0.5 and 2.0
        if(increasing) {
            contrastValue += 0.1f;
            if(contrastValue >= 2.0f) {
                increasing = false;
            }
        } else {
            contrastValue -= 0.1f;
            if(contrastValue <= 0.5f) {
                increasing = true;
            }
        }

        // Convert uint8_pixel_t image to bgr888_pixel_t image for USB
        convertUint8ToBgr888(dst, usb);

        // Copy timestamp
        ms2 = ms;

        // ---------------------------------------------------------------------
        // Set flag for USB interface that a new frame is available
        // ---------------------------------------------------------------------
        image_available_for_usb = 1;

        // Print debug info
        PRINTF("Contrast: %.1f | Processing time: %d ms\r\n", contrastValue, ms2-ms1);
    }
}

void exampleScaleFast(void)
{
    PRINTF("%s\r\n", __func__);

    // Update SysTick to increase precision to 0.01ms (=10us)
    uint32_t status = SysTick_Config(SystemCoreClock/100000);

    if(status != 0)
    {
        PRINTF("SysTick update failed\r\n");
        while(1)
        {}
    }

    // -------------------------------------------------------------------------
    // Local image memory allocation
    // -------------------------------------------------------------------------
    image_t *src = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT);
    image_t *dst = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT);

    if(dst == NULL)
    {
        PRINTF("Could not allocate image memory\r\n");
        while(1)
        {}
    }

    while(1U)
    {
        // ---------------------------------------------------------------------
        // Wait for camera image complete
        // ---------------------------------------------------------------------
        while(smartdma_camera_image_complete == 0)
        {}

        smartdma_camera_image_complete = 0;

        // ---------------------------------------------------------------------
        // Image processing pipeline
        // ---------------------------------------------------------------------
        // Convert camera image to uint8
        convertUyvyToUint8(cam, src);

        // Time the scaleFast operation
        ms1 = ms;
        scaleFast(src, dst);
        ms2 = ms;

        // Convert result to BGR888 for display
        convertUint8ToBgr888(dst, usb);

        // ---------------------------------------------------------------------
        // Set flag for USB interface that a new frame is available
        // ---------------------------------------------------------------------
        image_available_for_usb = 1;

        // Print execution time
        PRINTF("ScaleFast execution time: %d us\r\n", (ms2-ms1)*10);
    }
}

void exampleScale(void)
{
    PRINTF("%s\r\n", __func__);

    // Update SysTick to increase precision to 0.01ms (=10us)
    uint32_t status = SysTick_Config(SystemCoreClock/100000);

    if(status != 0)
    {
        PRINTF("SysTick update failed\r\n");
        while(1)
        {}
    }

    // -------------------------------------------------------------------------
    // Local image memory allocation
    // -------------------------------------------------------------------------
    image_t *src = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT);
    image_t *dst = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT);

    if(dst == NULL)
    {
        PRINTF("Could not allocate image memory\r\n");
        while(1)
        {}
    }

    while(1U)
    {
        // ---------------------------------------------------------------------
        // Wait for camera image complete
        // ---------------------------------------------------------------------
        while(smartdma_camera_image_complete == 0)
        {}

        smartdma_camera_image_complete = 0;

        // ---------------------------------------------------------------------
        // Image processing pipeline
        // ---------------------------------------------------------------------
        // Convert camera image to uint8
        convertUyvyToUint8(cam, src);

        // Time the scale operation
        ms1 = ms;
        scale(src, dst);
        ms2 = ms;

        // Convert result to BGR888 for display
        convertUint8ToBgr888(dst, usb);

        // ---------------------------------------------------------------------
        // Set flag for USB interface that a new frame is available
        // ---------------------------------------------------------------------
        image_available_for_usb = 1;

        // Print execution time
        PRINTF("Scale execution time: %d us\r\n", (ms2-ms1)*10);
    }
}

void exampleClearUint8(void)
{
    PRINTF("%s\r\n", __func__);

    // Update SysTick to increase precision to 0.01ms (=10us)
    uint32_t status = SysTick_Config(SystemCoreClock/100000);

    if(status != 0)
    {
        PRINTF("SysTick update failed\r\n");
        while(1)
        {}
    }

    // -------------------------------------------------------------------------
    // Local image memory allocation
    // -------------------------------------------------------------------------
    image_t *img = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT);

    if(img == NULL)
    {
        PRINTF("Could not allocate image memory\r\n");
        while(1)
        {}
    }

    while(1U)
    {
        // ---------------------------------------------------------------------
        // Wait for camera image complete
        // ---------------------------------------------------------------------
        while(smartdma_camera_image_complete == 0)
        {}

        smartdma_camera_image_complete = 0;

        // ---------------------------------------------------------------------
        // Image processing pipeline
        // ---------------------------------------------------------------------
        ms1 = ms;
        clearUint8Image(img);
        ms2 = ms;

        // Convert result to BGR888 for display
        convertUint8ToBgr888(img, usb);

        // ---------------------------------------------------------------------
        // Set flag for USB interface that a new frame is available
        // ---------------------------------------------------------------------
        image_available_for_usb = 1;

        // Print execution time
        PRINTF("ClearUint8 execution time: %d us\r\n", (ms2-ms1)*10);
    }
}

void exampleClearUint8Cm33(void)
{
    PRINTF("%s\r\n", __func__);

    // Update SysTick to increase precision to 0.01ms (=10us)
    uint32_t status = SysTick_Config(SystemCoreClock/100000);

    if(status != 0)
    {
        PRINTF("SysTick update failed\r\n");
        while(1)
        {}
    }

    // -------------------------------------------------------------------------
    // Local image memory allocation
    // -------------------------------------------------------------------------
    image_t *img = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT);

    if(img == NULL)
    {
        PRINTF("Could not allocate image memory\r\n");
        while(1)
        {}
    }

    // Fill image with pattern to verify clearing
    for(int i = 0; i < img->rows * img->cols; i++) {
        ((uint8_t*)img->data)[i] = i & 0xFF;
    }

    while(1U)
    {
        // ---------------------------------------------------------------------
        // Wait for camera image complete
        // ---------------------------------------------------------------------
        while(smartdma_camera_image_complete == 0)
        {}

        smartdma_camera_image_complete = 0;

        // ---------------------------------------------------------------------
        // Image processing pipeline
        // ---------------------------------------------------------------------
        ms1 = ms;
        clearUint8Image_cm33(img);
        ms2 = ms;

        // Convert result to BGR888 for display
        convertUint8ToBgr888(img, usb);

        // ---------------------------------------------------------------------
        // Set flag for USB interface that a new frame is available
        // ---------------------------------------------------------------------
        image_available_for_usb = 1;

        // Print execution time
        PRINTF("ClearUint8_cm33 execution time: %d us\r\n", (ms2-ms1)*10);
    }
}

void exampleConvolveFast(void)
{
    PRINTF("%s\r\n", __func__);

    // Update SysTick to increase precision to 0.01ms (=10us)
    uint32_t status = SysTick_Config(SystemCoreClock/100000);

    if(status != 0)
    {
        PRINTF("SysTick update failed\r\n");
        while(1)
        {}
    }

    // -------------------------------------------------------------------------
    // Local image memory allocation
    // -------------------------------------------------------------------------
    image_t *uint8_src = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT);
    image_t *src = newInt16Image(EVDK5_WIDTH, EVDK5_HEIGHT);
    image_t *dst = newInt16Image(EVDK5_WIDTH, EVDK5_HEIGHT);
    image_t *uint8_dst = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT);

    // Define 3x3 mask for mean filter (all 1's)
    int16_pixel_t msk_data[3 * 3] = {
        1, 1, 1,
        1, 1, 1,
        1, 1, 1,
    };

    image_t msk = {
        .cols = 3,
        .rows = 3,
        .type = IMGTYPE_INT16,
        .data = (uint8_t *)msk_data,
    };

    if(src == NULL || dst == NULL || uint8_src == NULL || uint8_dst == NULL)
    {
        PRINTF("Could not allocate image memory\r\n");
        while(1)
        {}
    }

    while(1U)
    {
        // Wait for camera image complete
        while(smartdma_camera_image_complete == 0)
        {}
        smartdma_camera_image_complete = 0;

        // Image processing pipeline
        convertUyvyToUint8(cam, uint8_src);  // Camera to uint8
        
        // Convert uint8 source to int16 for convolution
        for(int i = 0; i < EVDK5_WIDTH * EVDK5_HEIGHT; i++) {
            ((int16_pixel_t*)src->data)[i] = ((uint8_pixel_t*)uint8_src->data)[i];
        }

        // Time the convolveFast operation
        ms1 = ms;

        // Apply convolution
        convolveFast(src, dst, &msk);

        // Copy timestamp
        ms2 = ms;

        // Scale int16 result back to uint8 for display
        scaleInt16ToUint8(dst, uint8_dst);
        
        // Convert to BGR888 for display
        convertUint8ToBgr888(uint8_dst, usb);

        // Set flag for USB interface that a new frame is available
        image_available_for_usb = 1;

        // Print execution time
        PRINTF("ConvolveFast execution time: %d us\r\n", (ms2-ms1)*10);
    }
}

void exampleMeanFast(void)
{
    PRINTF("%s\r\n", __func__);

    // Update SysTick to increase precision to 0.01ms (=10us)
    uint32_t status = SysTick_Config(SystemCoreClock/100000);

    if(status != 0)
    {
        PRINTF("SysTick update failed\r\n");
        while(1)
        {}
    }

    // -------------------------------------------------------------------------
    // Local image memory allocation
    // -------------------------------------------------------------------------
    image_t *src = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT);
    image_t *dst = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT);

    if(dst == NULL)
    {
        PRINTF("Could not allocate image memory\r\n");
        while(1)
        {}
    }

    while(1U)
    {
        // ---------------------------------------------------------------------
        // Wait for camera image complete
        // ---------------------------------------------------------------------
        while(smartdma_camera_image_complete == 0)
        {}

        smartdma_camera_image_complete = 0;

        // ---------------------------------------------------------------------
        // Image processing pipeline
        // ---------------------------------------------------------------------
        // Convert camera image to uint8
        convertUyvyToUint8(cam, src);

        // Time the meanFast operation
        ms1 = ms;
        meanFast(src, dst);
        ms2 = ms;

        // Convert result to BGR888 for display
        convertUint8ToBgr888(dst, usb);

        // ---------------------------------------------------------------------
        // Set flag for USB interface that a new frame is available
        // ---------------------------------------------------------------------
        image_available_for_usb = 1;

        // Print execution time
        PRINTF("MeanFast execution time: %d us\r\n", (ms2-ms1)*10);
    }
}

void exampleConvolve(void)
{
    PRINTF("%s\r\n", __func__);

    // -------------------------------------------------------------------------
    // Local image memory allocation
    // -------------------------------------------------------------------------
    image_t *src = newInt16Image(EVDK5_WIDTH, EVDK5_HEIGHT);
    image_t *dst = newInt16Image(EVDK5_WIDTH, EVDK5_HEIGHT);
    image_t *uint8_dst = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT); // Add this intermediate image
    
    // Create a 3x3 mask for testing (simple edge detection)
    int16_pixel_t msk_data[3 * 3] = {
        -1, -1, -1,
        -1,  8, -1,
        -1, -1, -1,
    };

    image_t msk = {
        .cols = 3,
        .rows = 3,
        .type = IMGTYPE_INT16,
        .data = (uint8_t *)msk_data,
    };

    if(src == NULL || dst == NULL || uint8_dst == NULL) // Updated null check
    {
        PRINTF("Could not allocate image memory\r\n");
        while(1)
        {}
    }

    while(1U)
    {
        // ---------------------------------------------------------------------
        // Wait for camera image complete
        // ---------------------------------------------------------------------
        while(smartdma_camera_image_complete == 0)
        {}

        smartdma_camera_image_complete = 0;

        // ---------------------------------------------------------------------
        // Image processing pipeline
        // ---------------------------------------------------------------------
        // Convert camera image to int16
        convertUyvyToInt16(cam, src);

        // Copy timestamp
        ms1 = ms;

        // Apply convolution
        convolve(src, dst, &msk);

        // Copy timestamp
        ms2 = ms;

        // Scale the int16 result back to uint8 for display
        scaleInt16ToUint8(dst, uint8_dst); // Use intermediate uint8 image

        // Convert uint8 to BGR888 for USB display
        convertUint8ToBgr888(uint8_dst, usb);  

        // ---------------------------------------------------------------------
        // Set flag for USB interface that a new frame is available
        // ---------------------------------------------------------------------
        image_available_for_usb = 1;

        // Print debug info
        PRINTF("%d | delta: %d ms\r\n", ms1, ms2-ms1);
    }
}

void exampleFinalAssignment(void)
{
    PRINTF("%s\r\n", __func__);
    
    // Update SysTick for better precision
    SysTick_Config(SystemCoreClock/100000);
    
    // ---------------------------------------------------------------
    // Local image memory allocation
    // ---------------------------------------------------------------
    image_t *src = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT);
    image_t *dst = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT);
    image_t *tmp = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT);
    image_t *labeled = newInt32Image(EVDK5_WIDTH, EVDK5_HEIGHT);
    
    if (src == NULL || dst == NULL || tmp == NULL || labeled == NULL)
    {
        PRINTF("Could not allocate image memory\r\n");
        while(1) {}
    }
    
    while(1U)
    {
        // ---------------------------------------------------------------
        // Wait for camera image complete
        // ---------------------------------------------------------------
        while(smartdma_camera_image_complete == 0) {}
        smartdma_camera_image_complete = 0;
        
        // ---------------------------------------------------------------
        // Image processing pipeline
        // ---------------------------------------------------------------
        // Convert UYVY camera image to uint8 grayscale
        convertUyvyToUint8(cam, src);
        
        // Start timing
        ms1 = ms;
        
        // Apply noise reduction with meanFast
        meanFast(src, dst);
        
        // Stretch contrast with scaleFast
        scaleFast(dst, dst);
        
        // Save a copy of the pre-processed image for display
        copyUint8Image(dst, tmp);
        
        // Apply thresholding using Otsu's method
        thresholdOtsu(dst, dst, BRIGHTNESS_DARK);
        
        // Scale binary values for display (0->0, 1->255)
        for (uint32_t i = 0; i < EVDK5_WIDTH * EVDK5_HEIGHT; i++) {
            tmp->data[i] = dst->data[i] * 255;
        }
        
        // Remove border blobs
        removeBorderBlobsTwoPass(dst, dst, CONNECTED_FOUR, 200);
        
        // Label connected components
        uint32_t objectCount = labelTwoPass(dst, labeled, CONNECTED_FOUR, 256);
        
        // End timing
        ms2 = ms;
        
        // Update display image with the post-processed binary image
        for (uint32_t i = 0; i < EVDK5_WIDTH * EVDK5_HEIGHT; i++) {
            tmp->data[i] = dst->data[i] * 255;
        }
        
        // Find the largest object
        uint32_t largestObjectLabel = 0;
        uint32_t largestObjectArea = 0;
        
        if (objectCount > 0) {
            // First pass to find the largest object
            for (uint32_t i = 1; i <= objectCount; ++i) {
                blobinfo_t tempBlob = {0};
                area(labeled, &tempBlob, i);
                
                if (tempBlob.area > largestObjectArea) {
                    largestObjectArea = tempBlob.area;
                    largestObjectLabel = i;
                }
            }
            
            // Process only the largest object
            if (largestObjectLabel > 0) {
                blobinfo_t blob = {0};
                
                // Calculate area, circularity and Hu moments
                area(labeled, &blob, largestObjectLabel);
                circularity(labeled, &blob, largestObjectLabel);
                huInvariantMoments(labeled, &blob, largestObjectLabel);
                
                // Classify shape based on circularity
                const char* shape = "Unknown";
                if (blob.circularity > 0.9) {
                    shape = "Circle";
                } else if (blob.circularity >= 0.75 && blob.circularity <= 0.9) {
                    shape = "Square";
                } else if (blob.circularity >= 0.5 && blob.circularity < 0.75) {
                    shape = "Triangle";
                }
                
                // Print metrics for largest object
                PRINTF("Time: %d us | Shape: %s | Area=%d Circ=%.3f Hu=[%.6f, %.6f, %.6f, %.6f]\r\n", 
                       (ms2-ms1)*10, shape, blob.area, blob.circularity,
                       blob.hu_moments[0], blob.hu_moments[1], 
                       blob.hu_moments[2], blob.hu_moments[3]);
            } else {
                PRINTF("Time: %d us | Shape: None | No objects detected\r\n", (ms2-ms1)*10);
            }
        } else {
            PRINTF("Time: %d us | Shape: None | No objects detected\r\n", (ms2-ms1)*10);
        }
        
        // Convert tmp image to BGR888 format for USB display
        convertToBgr888(tmp, usb);
        
        // ---------------------------------------------------------------
        // Set flag for USB interface that a new frame is available
        // ---------------------------------------------------------------
        image_available_for_usb = 1;
    }
}

void exampleMean(void)
{
    PRINTF("%s\r\n", __func__);

    // Update SysTick to increase precision to 0.01ms (=10us)
    uint32_t status = SysTick_Config(SystemCoreClock/100000);

    if(status != 0)
    {
        PRINTF("SysTick update failed\r\n");
        while(1)
        {}
    }

    // -------------------------------------------------------------------------
    // Local image memory allocation
    // -------------------------------------------------------------------------
    image_t *src = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT);
    image_t *dst = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT);

    if(dst == NULL)
    {
        PRINTF("Could not allocate image memory\r\n");
        while(1)
        {}
    }

    while(1U)
    {
        // ---------------------------------------------------------------------
        // Wait for camera image complete
        // ---------------------------------------------------------------------
        while(smartdma_camera_image_complete == 0)
        {}

        smartdma_camera_image_complete = 0;

        // ---------------------------------------------------------------------
        // Image processing pipeline
        // ---------------------------------------------------------------------
        // Convert camera image to uint8
        convertUyvyToUint8(cam, src);

        // Time the mean operation
        ms1 = ms;
        mean(src, dst, 3); // Apply 3x3 mean filter
        ms2 = ms;

        // Convert result to BGR888 for display
        convertUint8ToBgr888(dst, usb);

        // ---------------------------------------------------------------------
        // Set flag for USB interface that a new frame is available
        // ---------------------------------------------------------------------
        image_available_for_usb = 1;

        // Print execution time
        PRINTF("Mean execution time: %d us\r\n", (ms2-ms1)*10);
    }
}

void exampleThreshold2Means(void)
{
    PRINTF("%s\r\n", __func__);

    // Update SysTick to increase precision to 0.01ms (=10us)
    uint32_t status = SysTick_Config(SystemCoreClock/100000);

    if(status != 0)
    {
        PRINTF("SysTick update failed\r\n");
        while(1)
        {}
    }

    // -------------------------------------------------------------------------
    // Local image memory allocation
    // -------------------------------------------------------------------------
    image_t *src = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT);
    image_t *dst = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT);

    if(dst == NULL)
    {
        PRINTF("Could not allocate image memory\r\n");
        while(1)
        {}
    }

    while(1U)
    {
        // ---------------------------------------------------------------------
        // Wait for camera image complete
        // ---------------------------------------------------------------------
        while(smartdma_camera_image_complete == 0)
        {}

        smartdma_camera_image_complete = 0;

        // ---------------------------------------------------------------------
        // Image processing pipeline
        // ---------------------------------------------------------------------
        // Convert camera image to uint8
        convertUyvyToUint8(cam, src);

        // Time the threshold2Means operation
        ms1 = ms;
        threshold2Means(src, dst, 1); // Apply 2-means thresholding
        ms2 = ms;

        // Convert result to BGR888 for display
        convertUint8ToBgr888(dst, usb);

        // ---------------------------------------------------------------------
        // Set flag for USB interface that a new frame is available
        // ---------------------------------------------------------------------
        image_available_for_usb = 1;

        // Print execution time
        PRINTF("Threshold2Means execution time: %d us\r\n", (ms2-ms1)*10);
    }
}

void exampleThresholdOtsu(void)
{
    PRINTF("%s\r\n", __func__);

    // Update SysTick to increase precision to 0.01ms (=10us)
    uint32_t status = SysTick_Config(SystemCoreClock/100000);

    if(status != 0)
    {
        PRINTF("SysTick update failed\r\n");
        while(1)
        {}
    }

    // -------------------------------------------------------------------------
    // Local image memory allocation
    // -------------------------------------------------------------------------
    image_t *src = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT);
    image_t *dst = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT);

    if(dst == NULL)
    {
        PRINTF("Could not allocate image memory\r\n");
        while(1)
        {}
    }

    while(1U)
    {
        // ---------------------------------------------------------------------
        // Wait for camera image complete
        // ---------------------------------------------------------------------
        while(smartdma_camera_image_complete == 0)
        {}

        smartdma_camera_image_complete = 0;

        // ---------------------------------------------------------------------
        // Image processing pipeline
        // ---------------------------------------------------------------------
        // Convert camera image to uint8
        convertUyvyToUint8(cam, src);

        // Time the thresholdOtsu operation
        ms1 = ms;
        thresholdOtsu(src, dst, 1); // Apply Otsu thresholding
        ms2 = ms;

        // Convert result to BGR888 for display
        convertUint8ToBgr888(dst, usb);

        // ---------------------------------------------------------------------
        // Set flag for USB interface that a new frame is available
        // ---------------------------------------------------------------------
        image_available_for_usb = 1;

        // Print execution time
        PRINTF("ThresholdOtsu execution time: %d us\r\n", (ms2-ms1)*10);
    }
}

void exampleSobelFast(void)
{
    PRINTF("%s\r\n", __func__);

    // Update SysTick to increase precision to 0.01ms (=10us)
    uint32_t status = SysTick_Config(SystemCoreClock/100000);

    if(status != 0)
    {
        PRINTF("SysTick update failed\r\n");
        while(1)
        {}
    }

    // -------------------------------------------------------------------------
    // Local image memory allocation
    // -------------------------------------------------------------------------
    image_t *src = newInt16Image(EVDK5_WIDTH, EVDK5_HEIGHT);
    image_t *mag = newInt16Image(EVDK5_WIDTH, EVDK5_HEIGHT);
    image_t *uint8_dst = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT);

    if(src == NULL || mag == NULL || uint8_dst == NULL)
    {
        PRINTF("Could not allocate image memory\r\n");
        while(1)
        {}
    }

    while(1U)
    {
        // ---------------------------------------------------------------------
        // Wait for camera image complete
        // ---------------------------------------------------------------------
        while(smartdma_camera_image_complete == 0)
        {}

        smartdma_camera_image_complete = 0;

        // ---------------------------------------------------------------------
        // Image processing pipeline
        // ---------------------------------------------------------------------
        // Convert camera image to int16
        convertUyvyToInt16(cam, src);

        // Time the sobelFast operation
        ms1 = ms;
        sobelFast(src, mag);
        ms2 = ms;

        // Scale int16 result back to uint8 for display
        scaleInt16ToUint8(mag, uint8_dst);

        // Convert to BGR888 for display
        convertUint8ToBgr888(uint8_dst, usb);

        // ---------------------------------------------------------------------
        // Set flag for USB interface that a new frame is available
        // ---------------------------------------------------------------------
        image_available_for_usb = 1;

        // Print execution time in microseconds
        PRINTF("SobelFast execution time: %04d us\r\n", (ms2-ms1)*10);
    }
}

void exampleRemoveBorderBlobs(void)
{
    PRINTF("%s\r\n", __func__);

    // Update SysTick for microsecond precision
    SysTick_Config(SystemCoreClock/100000);

    // Local image memory allocation
    image_t *src = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT);
    image_t *dst = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT);
    image_t *labeled = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT);

    if(src == NULL || dst == NULL || labeled == NULL)
    {
        PRINTF("Could not allocate image memory\r\n");
        while(1) {}
    }

    while(1U)
    {
        while(smartdma_camera_image_complete == 0) {}
        smartdma_camera_image_complete = 0;

        // Convert camera to uint8 and threshold to binary
        convertUyvyToUint8(cam, src);
        thresholdOtsu(src, dst, 1);

        // Time the operation
        ms1 = ms;
        removeBorderBlobsTwoPass(dst, labeled, CONNECTED_FOUR, 64);
        ms2 = ms;

        // Display result
        convertUint8ToBgr888(labeled, usb);
        image_available_for_usb = 1;

        PRINTF("RemoveBorderBlobs execution time: %d us\r\n", (ms2-ms1)*10);
    }
}

void exampleFillHoles(void)
{
    PRINTF("%s\r\n", __func__);

    // Update SysTick for microsecond precision
    SysTick_Config(SystemCoreClock/100000);

    // Local image memory allocation
    image_t *src = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT);
    image_t *binary = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT);
    image_t *filled = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT);

    if(src == NULL || binary == NULL || filled == NULL)
    {
        PRINTF("Could not allocate image memory\r\n");
        while(1) {}
    }

    while(1U)
    {
        while(smartdma_camera_image_complete == 0) {}
        smartdma_camera_image_complete = 0;

        // Convert camera to uint8 and threshold to binary
        convertUyvyToUint8(cam, src);
        thresholdOtsu(src, binary, 1);

        // Time the operation
        ms1 = ms;
        fillHolesTwoPass(binary, filled, CONNECTED_FOUR, 64);
        ms2 = ms;

        // Display result
        convertUint8ToBgr888(filled, usb);
        image_available_for_usb = 1;

        PRINTF("FillHoles execution time: %d us\r\n", (ms2-ms1)*10);
    }
}

void exampleLabel(void)
{
    PRINTF("%s\r\n", __func__);

    // Update SysTick for microsecond precision
    SysTick_Config(SystemCoreClock/100000);

    // Local image memory allocation
    image_t *src = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT);
    image_t *binary = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT);
    image_t *labeled = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT);

    if(src == NULL || binary == NULL || labeled == NULL)
    {
        PRINTF("Could not allocate image memory\r\n");
        while(1) {}
    }

    while(1U)
    {
        while(smartdma_camera_image_complete == 0) {}
        smartdma_camera_image_complete = 0;

        // Convert camera to uint8 and threshold to binary
        convertUyvyToUint8(cam, src);
        thresholdOtsu(src, binary, 1);

        // Time the operation
        ms1 = ms;
        uint8_t numLabels = labelTwoPass(binary, labeled, CONNECTED_FOUR, 64);
        ms2 = ms;

        // Display result
        convertUint8ToBgr888(labeled, usb);
        image_available_for_usb = 1;

        PRINTF("Label execution time: %d us, Found %d objects\r\n", 
               (ms2-ms1)*10, numLabels);
    }
}

void examplePerimeter(void)
{
    PRINTF("%s\r\n", __func__);

    // Update SysTick for microsecond precision
    SysTick_Config(SystemCoreClock/100000);

    // Local image memory allocation
    image_t *src = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT);
    image_t *binary = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT);
    image_t *labeled = newUint8Image(EVDK5_WIDTH, EVDK5_HEIGHT);
    blobinfo_t blobinfo = {0};  // Structure to store blob information

    if(src == NULL || binary == NULL || labeled == NULL)
    {
        PRINTF("Could not allocate image memory\r\n");
        while(1) {}
    }

    while(1U)
    {
        while(smartdma_camera_image_complete == 0) {}
        smartdma_camera_image_complete = 0;

        // Convert camera to uint8 and threshold to binary
        convertUyvyToUint8(cam, src);
        thresholdOtsu(src, binary, 1);

        // Label the binary image first
        uint8_t numLabels = labelTwoPass(binary, labeled, CONNECTED_FOUR, 64);

        if(numLabels > 0)
        {
            // Time the operation
            ms1 = ms;
            
            // Initialize blobinfo structure
            blobinfo.area = 0;
            blobinfo.perimeter = 0;
            blobinfo.circularity = 0;
            
            // Calculate perimeter using the optimized implementation
            perimeter(labeled, &blobinfo, 1);  // Get perimeter of blob #1
            
            ms2 = ms;

            // Convert perimeter result to display format
            for(int i = 0; i < EVDK5_WIDTH * EVDK5_HEIGHT; i++)
            {
                ((uint8_pixel_t*)labeled->data)[i] = (((uint8_pixel_t*)labeled->data)[i] == 1) ? 255 : 0;
            }
        }

        // Display result
        convertUint8ToBgr888(labeled, usb);
        image_available_for_usb = 1;

        // Print execution time and perimeter info
        PRINTF("Perimeter execution time: %d us, Perimeter length: %.2f\r\n", 
               (ms2-ms1)*10, blobinfo.perimeter);
    }
}
