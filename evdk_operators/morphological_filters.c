/*! ***************************************************************************
 *
 * \brief     Smooth the contours of objects and decompose an image into its
 *            fundamental geometrical shape
 * \file      morphological_filters.c
 * \author    Hugo Arends - HAN Embedded Vision and Machine Learning
 * \author
 * \date      October 2024
 *
 * \see       Myler, H. R., & Weeks, A. R. (2009). The pocket handbook of
 *            image processing algorithms in C. Prentice Hall Press.
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
#include "image_fundamentals.h"
#include "morphological_filters.h"

#include <string.h>

/*!
 * \brief Binary dilation of an object increases its geometrical area
 *
 * Dilation is defined as the union of all vector additions of all pixels a
 * in object A with all pixels b in the structuring function B (\p mask).
 *
 * \param[in]  src  A pointer to the source image
 * \param[out] dst  A pointer to the destination image
 * \param[in]  mask A pointer to a square mask of size \p n
 * \param[in]  n    The size of the mask
 */
void dilation(const image_t *src, image_t *dst, const uint8_t *mask, const uint8_t n)
{
    // Verify image validity
    ASSERT(src == NULL, "src image is invalid");
    ASSERT(dst == NULL, "dst image is invalid");
    ASSERT(src->data == NULL, "src data is invalid");
    ASSERT(dst->data == NULL, "dst data is invalid");
    ASSERT(src->type != IMGTYPE_UINT8, "src type is invalid");
    ASSERT(dst->type != IMGTYPE_UINT8, "dst type is invalid");

    // Verifiy mask validity
    ASSERT(mask == NULL, "mask is invalid");
    ASSERT(n == 0, "mask size can not be equal to 0");

    // Verify image consistency
    ASSERT(src == dst, "src and dst are the same images");
    ASSERT(src->cols != dst->cols, "src and dst have different number of columns");
    ASSERT(src->rows != dst->rows, "src and dst have different number of rows");

    // Loop all pixels
    for(int32_t y=0; y<src->rows; y++)
    {
        for(int32_t x=0; x<src->cols; x++)
        {
            uint32_t smax = 0;

            // Apply the kernel only for pixels within the image
            for(int32_t j=-n/2; j<=n/2; j++)
            {
                for(int32_t i=-n/2; i<=n/2; i++)
                {
                    if((x+i) >= 0 &&
                       (y+j) >= 0 &&
                       (x+i) <  src->cols &&
                       (y+j) <  src->rows)
                    {
                        // Is the pixel set and is the corresponding
                        // cell in the mask set?
                        if((getUint8Pixel(src,x+i,y+j) == 1) &&
                           (mask[((j+(n/2))*n) + (i+(n/2))] == 1))
                        {
                            // Mark this cell for dilation
                            smax = 1;
                        }
                    }
                }
            }

            // Store the result
            setUint8Pixel(dst,x,y,smax);
        }
    }
}

/*!
 * \brief Graylevel dilation is used to smooth small negative graylevel regions
 *
 * Graylevel dilation is defined as the maximum of the sum of a local region of
 * an image and a given graylevel \p mask.
 *
 * \param[in]  src  A pointer to the source image
 * \param[out] dst  A pointer to the destination image
 * \param[in]  mask A pointer to a square mask of size \p n
 * \param[in]  n    The size of the mask
 */
void dilationGray(const image_t *src, image_t *dst, const uint8_t *mask, const uint8_t n)
{
    // Verify image validity
    ASSERT(src == NULL, "src image is invalid");
    ASSERT(dst == NULL, "dst image is invalid");
    ASSERT(src->data == NULL, "src data is invalid");
    ASSERT(dst->data == NULL, "dst data is invalid");
    ASSERT(src->type != IMGTYPE_UINT8, "src type is invalid");
    ASSERT(dst->type != IMGTYPE_UINT8, "dst type is invalid");

    // Verifiy mask validity
    ASSERT(mask == NULL, "mask is invalid");
    ASSERT(n == 0, "mask size can not be equal to 0");

    // Verify image consistency
    ASSERT(src == dst, "src and dst are the same images");
    ASSERT(src->cols != dst->cols, "src and dst have different number of columns");
    ASSERT(src->rows != dst->rows, "src and dst have different number of rows");

    // Loop all pixels
    for(int32_t y=0; y<src->rows; y++)
    {
        for(int32_t x=0; x<src->cols; x++)
        {
            int32_t smax = 0;

            // Apply the kernel only for pixels within the image
            for(int32_t j=-n/2; j<=n/2; j++)
            {
                for(int32_t i=-n/2; i<=n/2; i++)
                {
                    if((x+i) >= 0 &&
                       (y+j) >= 0 &&
                       (x+i) <  src->cols &&
                       (y+j) <  src->rows)
                    {
                        int32_t val = getUint8Pixel(src,x+i,y+j) + mask[((j+(n/2))*n) + (i+(n/2))];

                        if(val > smax)
                            smax = val;
                    }
                }
            }

            // Clip the result
            if(smax > 255)
            {
                smax = 255;
            }

            // Store the result
            setUint8Pixel(dst,x,y,smax);
        }
    }
}

/*!
 * \brief Binary erosion of an object decreases its geometrical area
 *
 * Erosion is defined as the complement of the resulting dilation of the
 * complement of object A with structuring function B (\p mask).
 *
 * \param[in]  src  A pointer to the source image
 * \param[out] dst  A pointer to the destination image
 * \param[in]  mask A pointer to a square mask of size \p n
 * \param[in]  n    The size of the mask
 */
void erosion(const image_t *src, image_t *dst, const uint8_t *mask, const uint8_t n)
{
    // Verify image validity
    ASSERT(src == NULL, "src image is invalid");
    ASSERT(dst == NULL, "dst image is invalid");
    ASSERT(src->data == NULL, "src data is invalid");
    ASSERT(dst->data == NULL, "dst data is invalid");
    ASSERT(src->type != IMGTYPE_UINT8, "src type is invalid");
    ASSERT(dst->type != IMGTYPE_UINT8, "dst type is invalid");

    // Verifiy mask validity
    ASSERT(mask == NULL, "mask is invalid");
    ASSERT(n == 0, "mask size can not be equal to 0");

    // Verify image consistency
    ASSERT(src == dst, "src and dst are the same images");
    ASSERT(src->cols != dst->cols, "src and dst have different number of columns");
    ASSERT(src->rows != dst->rows, "src and dst have different number of rows");

    // Loop all pixels
    for(int32_t y=0; y<src->rows; y++)
    {
        for(int32_t x=0; x<src->cols; x++)
        {
            uint32_t smin = 1;

            // Apply the kernel only for pixels within the image
            for(int32_t j=-n/2; j<=n/2; j++)
            {
                for(int32_t i=-n/2; i<=n/2; i++)
                {
                    if((x+i) >= 0 &&
                       (y+j) >= 0 &&
                       (x+i) <  src->cols &&
                       (y+j) <  src->rows)
                    {
                        // Is the pixel background and is the corresponding
                        // cell in the mask set?
                        if((getUint8Pixel(src,x+i,y+j) == 0) &&
                           (mask[((j+(n/2))*n) + (i+(n/2))] == 1))
                        {
                            // Mark this cell for erosion
                            smin = 0;
                        }
                    }
                }
            }

            // Store the result
            setUint8Pixel(dst,x,y,smin);
        }
    }
}

/*!
 * \brief Graylevel dilation is used to smooth small positive graylevel regions
 *
 * Graylevel dilation is defined as the minimum of the difference of a local
 * region of an image and a given graylevel \p mask.
 *
 * \param[in]  src  A pointer to the source image
 * \param[out] dst  A pointer to the destination image
 * \param[in]  mask A pointer to a square mask of size \p n
 * \param[in]  n    The size of the mask
 */
void erosionGray(const image_t *src, image_t *dst, const uint8_t *mask, const uint8_t n)
{
    // Verify image validity
    ASSERT(src == NULL, "src image is invalid");
    ASSERT(dst == NULL, "dst image is invalid");
    ASSERT(src->data == NULL, "src data is invalid");
    ASSERT(dst->data == NULL, "dst data is invalid");
    ASSERT(src->type != IMGTYPE_UINT8, "src type is invalid");
    ASSERT(dst->type != IMGTYPE_UINT8, "dst type is invalid");

    // Verifiy mask validity
    ASSERT(mask == NULL, "mask is invalid");
    ASSERT(n == 0, "mask size can not be equal to 0");

    // Verify image consistency
    ASSERT(src == dst, "src and dst are the same images");
    ASSERT(src->cols != dst->cols, "src and dst have different number of columns");
    ASSERT(src->rows != dst->rows, "src and dst have different number of rows");

    // Loop all pixels
    for(int32_t y=0; y<src->rows; y++)
    {
        for(int32_t x=0; x<src->cols; x++)
        {
            int32_t smin = 255;

            // Apply the kernel only for pixels within the image
            for(int32_t j=-n/2; j<=n/2; j++)
            {
                for(int32_t i=-n/2; i<=n/2; i++)
                {
                    if((x+i) >= 0 &&
                       (y+j) >= 0 &&
                       (x+i) <  src->cols &&
                       (y+j) <  src->rows)
                    {
                        int32_t val = getUint8Pixel(src,x+i,y+j) - mask[((j+(n/2))*n) + (i+(n/2))];

                        if(val < smin)
                            smin = val;
                    }
                }
            }

            // Clip the result
            if(smin < 0)
            {
                smin = 0;
            }

            // Store the result
            setUint8Pixel(dst,x,y,smin);
        }
    }
}

/*!
 * \brief Fills the holes of a binary object
 *
 * Connectivity is as seen from the hole. If the hole is 4-connected, the
 * object’s boundary is 8-connected and vice versa.
 *
 * \param[in]  src A pointer to the source image
 * \param[out] dst A pointer to the destination image
 * \param[in]  c   The hole's connectivity. Must be of type ::eConnected.
 */
void fillHolesIterative(const image_t *src, image_t *dst, const eConnected c)
{
    // Verify image validity
    ASSERT(src == NULL, "src image is invalid");
    ASSERT(dst == NULL, "dst image is invalid");
    ASSERT(src->data == NULL, "src data is invalid");
    ASSERT(dst->data == NULL, "dst data is invalid");
    ASSERT(src->type != IMGTYPE_UINT8, "src type is invalid");
    ASSERT(dst->type != IMGTYPE_UINT8, "dst type is invalid");

    // Verify image consistency
    ASSERT(src->cols != dst->cols, "src and dst have different number of columns");
    ASSERT(src->rows != dst->rows, "src and dst have different number of rows");

    // Copy the image if needed
    if(src != dst)
    {
        copyUint8Image(src, dst);
    }

    // Mark the border pixels that are set
    for(int32_t y=0; y<dst->rows; y++)
    {
        if(getUint8Pixel(dst,0,y) == 0)
        {
            setUint8Pixel(dst,0,y,2);
        }

        if(getUint8Pixel(dst,(dst->cols-1),y) == 0)
        {
            setUint8Pixel(dst,(dst->cols-1),y,2);
        }
    }

    for(int32_t x=0; x<dst->cols; x++)
    {
        if(getUint8Pixel(dst,x,0) == 0)
        {
            setUint8Pixel(dst,x,0,2);
        }

        if(getUint8Pixel(dst,x,(dst->rows-1)) == 0)
        {
            setUint8Pixel(dst,x,(dst->rows-1),2);
        }
    }

    uint32_t changes = 0;
    uint32_t iterations = 0;

    // Set all 0 pixels that are connected to an edge to the marker value (2)
    do
    {
        // Loop as long as there are changes
        changes = 0;

        // Pass through from left-top to right-bottom
        // Discard first row, last row, first column and last column
        for(int32_t y = 1; y < dst->rows-1; y++)
        {
            for(int32_t x = 1; x < dst->cols-1; x++)
            {
                // Does the pixel belong to the background?
                if(getUint8Pixel(dst,x,y) == 0)
                {
                    // Is a neighbour marked?
                    if(neighbourCount(dst,x,y,2,c) > 0)
                    {
                        // Also mark this pixel
                        setUint8Pixel(dst,x,y,2);
                        changes = 1;
                    }
                }
            }
        }

        iterations++;

        if(!changes)
            break;

        // Pass through from right-bottom to left-top
        // Discard first row, last row, firs column and last column
        for(int32_t y = dst->rows-2; y > 0 ; y--)
        {
            for(int32_t x = dst->cols-2; x > 0; x--)
            {
                // Does the pixel belong to the background?
                if(getUint8Pixel(dst,x,y) == 0)
                {
                    // Is a neighbour marked?
                    if(neighbourCount(dst,x,y,2,c) > 0)
                    {
                        // Also mark this pixel
                        setUint8Pixel(dst,x,y,2);
                        changes = 1;
                    }
                }
            }
        }

        iterations++;
    }
    while(changes != 0);

    // Set background to object value
    for(int32_t y=0; y<src->rows; y++)
    {
        for(int32_t x=0; x<src->cols; x++)
        {
            if(getUint8Pixel(dst,x,y) == 0)
            {
                setUint8Pixel(dst,x,y,1);
            }
        }
    }

    // Set marked value to background value
    for(int32_t y=0; y<src->rows; y++)
    {
        for(int32_t x=0; x<src->cols; x++)
        {
            if(getUint8Pixel(dst,x,y) == 2)
            {
                setUint8Pixel(dst,x,y,0);
            }
        }
    }
}

/*!
 * \brief Fills the holes of a binary object
 *
 * Connectivity is as seen from the hole. If the hole is 4-connected, the
 * object’s boundary is 8-connected and vice versa.
 *
 * \param[in]  src     A pointer to the source image
 * \param[out] dst     A pointer to the destination image
 * \param[in]  c       Connectivity defined by ::eConnected
 * \param[in]  lutSize   The maximum number of labels to find. Is used to
 *                       dynamically allocate memory for the lookup table. A
 *                       lower value is faster, but is also able to find fewer
 *                       labels. The value is application dependent.
 *
 * \return 0 Failure
 *           - Memory allocation failed
 *           - Lookup table is too small
 *         1 Success
 *
 * \todo Implement this function
 */
uint32_t fillHolesTwoPass(const image_t *src, image_t *dst,
                          const eConnected connected, const uint32_t lutSize)
{
    // Verify input parameters
    if (src == NULL || dst == NULL || src->data == NULL || dst->data == NULL)
    {
        return 0; // Invalid input images
    }

    // Get image dimensions
    const uint32_t width = src->cols;
    const uint32_t height = src->rows;
    const uint32_t imageSize = width * height;

    // Allocate memory for label arrays
    uint32_t *labelMap = (uint32_t *)malloc(imageSize * sizeof(uint32_t));
    uint32_t *labelEquivalence = (uint32_t *)malloc(lutSize * sizeof(uint32_t));
    uint8_t *borderFlags = (uint8_t *)malloc(lutSize * sizeof(uint8_t));

    // Verify memory allocation
    if (labelMap == NULL || labelEquivalence == NULL || borderFlags == NULL)
    {
        free(labelMap);
        free(labelEquivalence);
        free(borderFlags);
        return 0; // Memory allocation failed
    }

    // Initialize work arrays
    memset(labelMap, 0, imageSize * sizeof(uint32_t));
    memset(borderFlags, 0, lutSize * sizeof(uint8_t));
    
    // Initialize label equivalence table
    for (uint32_t i = 0; i < lutSize; i++)
    {
        labelEquivalence[i] = i;
    }

    // Get direct pixel access
    const uint8_t *sourcePixel = (const uint8_t *)src->data;
    uint8_t *destinationPixel = (uint8_t *)dst->data;
    uint32_t currentLabelID = 1; // Start labeling from 1

    // First pass: Label all pixels and track connected components
    for (uint32_t y = 0; y < height; y++)
    {
        for (uint32_t x = 0; x < width; x++)
        {
            const uint32_t pixelPosition = y * width + x;
            
            // Skip foreground pixels
            if (sourcePixel[pixelPosition] == 1)
            {
                labelMap[pixelPosition] = 0;
                continue;
            }

            uint32_t minimalLabel = lutSize;
            uint8_t hasConnectedNeighbor = 0;

            // Check left neighbor
            if (x > 0 && labelMap[pixelPosition - 1] > 0)
            {
                minimalLabel = labelMap[pixelPosition - 1];
                hasConnectedNeighbor = 1;
            }

            // Check top neighbor
            if (y > 0 && labelMap[pixelPosition - width] > 0)
            {
                const uint32_t topLabel = labelMap[pixelPosition - width];
                if (hasConnectedNeighbor)
                {
                    // Track connected regions
                    if (topLabel < minimalLabel)
                    {
                        labelEquivalence[minimalLabel] = topLabel;
                        minimalLabel = topLabel;
                    }
                    else
                    {
                        labelEquivalence[topLabel] = minimalLabel;
                    }
                }
                else
                {
                    minimalLabel = topLabel;
                    hasConnectedNeighbor = 1;
                }
            }

            // For 8-connected mode, check diagonal neighbors
            if (connected == CONNECTED_EIGHT)
            {
                // Check top-left neighbor
                if (x > 0 && y > 0 && labelMap[pixelPosition - width - 1] > 0)
                {
                    const uint32_t topLeftLabel = labelMap[pixelPosition - width - 1];
                    if (hasConnectedNeighbor)
                    {
                        if (topLeftLabel < minimalLabel)
                        {
                            labelEquivalence[minimalLabel] = topLeftLabel;
                            minimalLabel = topLeftLabel;
                        }
                        else
                        {
                            labelEquivalence[topLeftLabel] = minimalLabel;
                        }
                    }
                    else
                    {
                        minimalLabel = topLeftLabel;
                        hasConnectedNeighbor = 1;
                    }
                }
                
                // Check top-right neighbor
                if (x < width - 1 && y > 0 && labelMap[pixelPosition - width + 1] > 0)
                {
                    const uint32_t topRightLabel = labelMap[pixelPosition - width + 1];
                    if (hasConnectedNeighbor)
                    {
                        if (topRightLabel < minimalLabel)
                        {
                            labelEquivalence[minimalLabel] = topRightLabel;
                            minimalLabel = topRightLabel;
                        }
                        else
                        {
                            labelEquivalence[topRightLabel] = minimalLabel;
                        }
                    }
                    else
                    {
                        minimalLabel = topRightLabel;
                        hasConnectedNeighbor = 1;
                    }
                }
            }

            // Assign new label if no connected neighbors found
            if (!hasConnectedNeighbor)
            {
                if (currentLabelID >= lutSize)
                {
                    free(labelMap);
                    free(labelEquivalence);
                    free(borderFlags);
                    return 0; // Label table overflow
                }
                minimalLabel = currentLabelID++;
            }
            
            labelMap[pixelPosition] = minimalLabel;
        }
    }

    // Resolve label equivalences
    for (uint32_t i = 1; i < currentLabelID; i++)
    {
        uint32_t rootLabel = i;
        while (labelEquivalence[rootLabel] != rootLabel)
        {
            rootLabel = labelEquivalence[rootLabel];
        }
        
        // Update all labels to point to their root
        uint32_t currentLabel = i;
        while (labelEquivalence[currentLabel] != rootLabel)
        {
            uint32_t nextLabel = labelEquivalence[currentLabel];
            labelEquivalence[currentLabel] = rootLabel;
            currentLabel = nextLabel;
        }
    }

    // Mark regions connected to image borders
    for (uint32_t x = 0; x < width; x++)
    {
        if (labelMap[x] > 0)
            borderFlags[labelEquivalence[labelMap[x]]] = 1;
        if (labelMap[(height - 1) * width + x] > 0)
            borderFlags[labelEquivalence[labelMap[(height - 1) * width + x]]] = 1;
    }

    for (uint32_t y = 0; y < height; y++)
    {
        if (labelMap[y * width] > 0)
            borderFlags[labelEquivalence[labelMap[y * width]]] = 1;
        if (labelMap[y * width + (width - 1)] > 0)
            borderFlags[labelEquivalence[labelMap[y * width + (width - 1)]]] = 1;
    }

    // Second pass: Fill holes and create final image
    for (uint32_t i = 0; i < imageSize; i++)
    {
        if (sourcePixel[i] == 1)
        {
            destinationPixel[i] = 1; // Preserve foreground pixels
        }
        else if (labelMap[i] > 0)
        {
            // Fill holes not connected to border
            if (borderFlags[labelEquivalence[labelMap[i]]] == 0)
            {
                destinationPixel[i] = 1;
            }
            else
            {
                destinationPixel[i] = 0;
            }
        }
        else
        {
            destinationPixel[i] = 0;
        }
    }

    // Release allocated memory
    free(labelMap);
    free(labelEquivalence);
    free(borderFlags);

    return 1; // Success
}

/*!
 * \brief This function is used to find geometrical features
 *
 * The function uses a hit mask and a miss mask with the requirement that the
 * intersection of the two masks is empty.
 *
 * \param[in]  src A pointer to the source image
 * \param[out] dst A pointer to the destination image
 * \param[in]  m1  3x3 Hit mask
 * \param[in]  m2  3x3 Miss mask
 */
void hitmiss(const image_t *src, image_t *dst, const uint8_t *m1, const uint8_t *m2)
{
    // Verify image validity
    ASSERT(src == NULL, "src image is invalid");
    ASSERT(dst == NULL, "dst image is invalid");
    ASSERT(src->data == NULL, "src data is invalid");
    ASSERT(dst->data == NULL, "dst data is invalid");
    ASSERT(src->type != IMGTYPE_UINT8, "src type is invalid");
    ASSERT(dst->type != IMGTYPE_UINT8, "dst type is invalid");

    // Verifiy mask validity
    ASSERT((m1[0] & m2[0]) == 1 ||
               (m1[1] & m2[1]) == 1 ||
               (m1[2] & m2[2]) == 1 ||
               (m1[3] & m2[3]) == 1 ||
               (m1[4] & m2[4]) == 1 ||
               (m1[5] & m2[5]) == 1 ||
               (m1[6] & m2[6]) == 1 ||
               (m1[7] & m2[7]) == 1 ||
               (m1[8] & m2[8]) == 1,
           " m1 AND m2 must be 0");

    // Verify image consistency
    ASSERT(src == dst, "src and dst are the same images");
    ASSERT(src->cols != dst->cols, "src and dst have different number of columns");
    ASSERT(src->rows != dst->rows, "src and dst have different number of rows");

    // Create temporary image
    image_t *org = newUint8Image(src->cols, src->rows);
    image_t *tmp = newUint8Image(src->cols, src->rows);

    ASSERT(tmp == NULL, "not enough memory for image allocation");

    copyUint8Image(src, org);

    // Set tmp image to the complement of org
    for(int32_t y=0; y<src->rows; y++)
    {
        for(int32_t x=0; x<src->cols; x++)
        {
            setUint8Pixel(tmp,x,y, 1 - getUint8Pixel(src,x,y));
        }
    }

    erosion(org, dst, m1, 3);
    erosion(tmp, org, m2, 3);

    // Calculate the intersection
    for(int32_t y=0; y<src->rows; y++)
    {
        for(int32_t x=0; x<src->cols; x++)
        {
            setUint8Pixel(dst,x,y, getUint8Pixel(dst,x,y) & getUint8Pixel(org,x,y));
        }
    }

    // Cleanup
    deleteUint8Image(org);
    deleteUint8Image(tmp);
}

/*!
 * \brief Change all of the object’s pixels to the background value, except
 * those pixels that lie on the object’s contour
 *
 * The contour width is determined by the structuring element \p mask.
 * The result is the eroded image subtracted from the original image or the
 * original image subtracted from the dilated image.
 *
 * \param[in]  src  A pointer to the source image
 * \param[out] dst  A pointer to the destination image
 * \param[in]  mask A pointer to a square mask of size \p n
 * \param[in]  n    The size of the mask
 */
void outline(const image_t *src, image_t *dst, const uint8_t *mask, const uint8_t n)
{
    // Verify image validity
    ASSERT(src == NULL, "src image is invalid");
    ASSERT(dst == NULL, "dst image is invalid");
    ASSERT(src->data == NULL, "src data is invalid");
    ASSERT(dst->data == NULL, "dst data is invalid");
    ASSERT(src->type != IMGTYPE_UINT8, "src type is invalid");
    ASSERT(dst->type != IMGTYPE_UINT8, "dst type is invalid");

    // Verifiy mask validity
    ASSERT(mask == NULL, "mask is invalid");
    ASSERT(n == 0, "mask size can not be equal to 0");

    // Verify image consistency
    ASSERT(src == dst, "src and dst are the same images");
    ASSERT(src->cols != dst->cols, "src and dst have different number of columns");
    ASSERT(src->rows != dst->rows, "src and dst have different number of rows");

    erosion(src, dst, mask, n);

    // Loop all pixels
    for(int32_t y=0; y<src->rows; y++)
    {
        for(int32_t x=0; x<src->cols; x++)
        {
            setUint8Pixel(dst,x,y, getUint8Pixel(src,x,y) - getUint8Pixel(dst,x,y));
        }
    }
}

/*!
 * \brief Removes all binary objects that are 4/8-connected to a border.
 *
 * \param[in]  src A pointer to the source image
 * \param[out] dst A pointer to the destination image
 * \param[in]  c   Connectivity defined by ::eConnected
 */
void removeBorderBlobsIterative(const image_t *src, image_t *dst, const eConnected c)
{
    // Verify image validity
    ASSERT(src == NULL, "src image is invalid");
    ASSERT(dst == NULL, "dst image is invalid");
    ASSERT(src->data == NULL, "src data is invalid");
    ASSERT(dst->data == NULL, "dst data is invalid");
    ASSERT(src->type != IMGTYPE_UINT8, "src type is invalid");
    ASSERT(dst->type != IMGTYPE_UINT8, "dst type is invalid");

    // Verify image consistency
    ASSERT(src == dst, "src and dst are the same images");
    ASSERT(src->cols != dst->cols, "src and dst have different number of columns");
    ASSERT(src->rows != dst->rows, "src and dst have different number of rows");

    copyUint8Image(src, dst);

    // Mark the border pixels that are set
    for(int32_t y=0; y<dst->rows; y++)
    {
        if(getUint8Pixel(dst,0,y) == 1)
        {
            setUint8Pixel(dst,0,y,2);
        }

        if(getUint8Pixel(dst,(dst->cols-1),y) == 1)
        {
            setUint8Pixel(dst,(dst->cols-1),y,2);
        }
    }

    for(int32_t x=0; x<dst->cols; x++)
    {
        if(getUint8Pixel(dst,x,0) == 1)
        {
            setUint8Pixel(dst,x,0,2);
        }

        if(getUint8Pixel(dst,x,(dst->rows-1)) == 1)
        {
            setUint8Pixel(dst,x,(dst->rows-1),2);
        }
    }

    uint32_t changes = 0;
    uint32_t iterations = 0;

    // Set all 1 pixels that are connected to an edge to the marker value (2)
    do
    {
        // Loop as long as there are changes
        changes = 0;

        // Pass through from left-top to right-bottom
        // Discard first row, last row, first column and last column
        for(int32_t y = 1; y < dst->rows-1; y++)
        {
            for(int32_t x = 1; x < dst->cols-1; x++)
            {
                // Does the pixel belong to a blob?
                if(getUint8Pixel(dst,x,y) == 1)
                {
                    // Is a neighbour marked?
                    if(neighbourCount(dst,x,y,2,c) > 0)
                    {
                        // Also mark this pixel
                        setUint8Pixel(dst,x,y,2);
                        changes = 1;
                    }
                }
            }
        }

        iterations++;
    }
    while(changes != 0);

    // Set marked value to background value
    for(int32_t y=0; y<src->rows; y++)
    {
        for(int32_t x=0; x<src->cols; x++)
        {
            if(getUint8Pixel(dst,x,y) == 2)
            {
                setUint8Pixel(dst,x,y,0);
            }
        }
    }
}

/*!
 * \brief Removes all binary objects that are 4/8-connected to a border.
 *
 * \param[in]  src     A pointer to the source image
 * \param[out] dst     A pointer to the destination image
 * \param[in]  c       Connectivity defined by ::eConnected
 * \param[in]  lutSize The maximum number of labels to find. Is used to
 *                     dynamically allocate memory for the lookup table. A
 *                     lower value is faster, but is also able to find fewer
 *                     labels. The value is application dependent.
 *
 * \return 0 Failure
 *           \li Memory allocation failed
 *           \li Lookup table is too small
 *         1 Success
 *
 * \todo Implement this function
 *
 * TIPS
        The argument lutSize is used to dynamically allocate memory in the removeBorderBlobsTwoPass() function for the equivalence lookup table.
            Use the function malloc().
            Check if the allocation succeeded by verifying if the returned pointer is not equal to NULL.
            Use the function memset() to set the entire lookup table to zero.
            Do not forget to use free() when the function finishes.
        The function returns 1 on successful execution and returns 0 in case of the following failures:
            Memory allocation for the lookup table failed
            The lookup table is too small. In other words, the image requires more unique labels than can be stored in the lookup table.
 *
 */
uint32_t removeBorderBlobsTwoPass(const image_t *src, image_t *dst, const eConnected connected, const uint32_t lutSize)
{
    // This function removes shapes that touch the edges of the picture.
    // First, we check if the pictures are valid.
    if (!src || !dst || !src->data || !dst->data)
    {
        return 2;
    }

    uint32_t width = src->cols;
    uint32_t height = src->rows;

    // Get direct pointers to pixel data for faster access
    const uint8_t *sourcePixel = (const uint8_t *)src->data;
    uint8_t *destinationPixel = (uint8_t *)dst->data;

    // We create a table of numbers to label each shape we find in the picture.
    // Shapes touching the border get a special label, so we can remove them.
    // Create arrays for storing labels and lookup table
    // LUT is used to track label equivalences
    uint32_t *labelEquivalence = (uint32_t *)calloc(lutSize, sizeof(uint32_t));
    uint32_t *labelMap = (uint32_t *)calloc(width * height, sizeof(uint32_t));

    // Check if memory allocation worked
    if (!labelMap || !labelEquivalence)
    {
        free(labelEquivalence);
        free(labelMap);
        return 0;
    }

    // Setup initial LUT values:
    // 1 = regular label
    // 2 = special label for border-connected pixels
    labelEquivalence[1] = 1;
    labelEquivalence[2] = 2;
    uint32_t currentLabelID = 3; // Start new labels from 3

    // Mark all object pixels that touch the image borders with label 2
    // Process top border
    for (uint32_t x = 0; x < width; ++x)
    {
        if (sourcePixel[x] == 1)
        {
            labelMap[x] = 2;
            destinationPixel[x] = 2;
        }
    }

    // Process bottom border
    for (uint32_t x = 0; x < width; ++x)
    {
        if (sourcePixel[(height - 1) * width + x] == 1)
        {
            labelMap[(height - 1) * width + x] = 2;
            destinationPixel[(height - 1) * width + x] = 2;
        }
    }

    // Process left border
    for (uint32_t y = 0; y < height; ++y)
    {
        if (sourcePixel[y * width] == 1)
        {
            labelMap[y * width] = 2;
            destinationPixel[y * width] = 2;
        }
    }

    // Process right border
    for (uint32_t y = 0; y < height; ++y)
    {
        if (sourcePixel[y * width + (width - 1)] == 1)
        {
            labelMap[y * width + (width - 1)] = 2;
            destinationPixel[y * width + (width - 1)] = 2;
        }
    }

    // We look at each pixel and see if it belongs to a shape.
    // If it does, we check its neighbors to figure out what label we give it.
    // First pass: Label all object pixels and track connected components
    for (uint32_t y = 0; y < height; ++y)
    {
        for (uint32_t x = 0; x < width; ++x)
        {

            // Here, we calculate the position of the current pixel in the image array.
            uint32_t pixelPosition = y * width + x;

            // We also calculate the positions of neighboring pixels above and to the left (and diagonals if 8-connected).
            uint32_t pixelUp;
            if (y > 0)
            {
                pixelUp = (y - 1) * width + x;
            }
            else
            {
                pixelUp = 0;
            }

            uint32_t pixelLeft;
            if (x > 0)
            {
                pixelLeft = y * width + (x - 1);
            }
            else
            {
                pixelLeft = 0;
            }

            uint32_t pixelTopLeft;
            if (x > 0 && y > 0)
            {
                pixelTopLeft = (y - 1) * width + (x - 1);
            }
            else
            {
                pixelTopLeft = 0;
            }

            uint32_t pixelTopRight;
            if (x < width - 1 && y > 0)
            {
                pixelTopRight = (y - 1) * width + (x + 1);
            }
            else
            {
                pixelTopRight = 0;
            }

            // Skip background pixels (value 0) because we only label object pixels.
            if (sourcePixel[pixelPosition] == 0)
            {
                destinationPixel[pixelPosition] = 0;
                continue;
            }

            // If this is the top-left corner, it has no neighbors to compare with.
            else if (x == 0 && y == 0)
            {
                continue;
            }

            // Gather neighbor labels in an array so we can find the smallest label among them.
            // This helps group pixels that belong to the same shape.
            uint32_t neighborLabels[5] = {0};
            int neighborCount = 0;

            // Check neighbors based on connectivity type (4 or 8 connected)
            if (y > 0)
            {
                neighborLabels[neighborCount++] = labelMap[pixelUp]; // Top
            }

            if (x > 0)
            {
                neighborLabels[neighborCount++] = labelMap[pixelLeft]; // Left
            }

            // For 8-connectivity, also check diagonal neighbors
            if (connected == CONNECTED_EIGHT)
            {
                if (y > 0 && x > 0)
                {
                    neighborLabels[neighborCount++] = labelMap[pixelTopLeft]; // Top-left
                }
                if (y > 0 && x < width - 1)
                {
                    neighborLabels[neighborCount++] = labelMap[pixelTopRight]; // Top-right
                }
            }

            // Include current pixel's label if it exists
            if (labelMap[pixelPosition] != 0)
            {
                neighborLabels[neighborCount++] = labelMap[pixelPosition];
            }

            // We find the smallest non-background label among neighbors to unify connected shapes.
            uint32_t minimalNeighborLabel = 0;
            for (int i = 0; i < neighborCount; ++i)
            {
                if (neighborLabels[i] != 0 && neighborLabels[i] != 1)
                {
                    if (minimalNeighborLabel == 0 || neighborLabels[i] < minimalNeighborLabel)
                    {
                        minimalNeighborLabel = neighborLabels[i];
                    }
                }
            }

            // If we cannot find a suitable label among neighbors, create a new label if there's still room in the LUT.
            if (minimalNeighborLabel == 0)
            {
                // Check if we have room for new label
                if (currentLabelID >= lutSize)
                {
                    free(labelEquivalence);
                    free(labelMap);
                    return 5;
                }

                labelMap[pixelPosition] = currentLabelID;
                labelEquivalence[currentLabelID] = currentLabelID;
                destinationPixel[pixelPosition] = currentLabelID;
                ++currentLabelID;
            }
            // If neighbors exist, use smallest label and update equivalences
            else
            {
                labelMap[pixelPosition] = minimalNeighborLabel;
                destinationPixel[pixelPosition] = minimalNeighborLabel;

                // Update label equivalences
                for (int i = 0; i < neighborCount; ++i)
                {
                    if (neighborLabels[i] != 0 && neighborLabels[i] != minimalNeighborLabel)
                    {
                        labelEquivalence[neighborLabels[i]] = minimalNeighborLabel;
                    }
                }

                // We also handle the case where a shape touches the image border by assigning it label 2.
                // This will be removed in the second pass.
                if (minimalNeighborLabel == 2 || labelEquivalence[minimalNeighborLabel] == 2)
                {
                    labelEquivalence[labelMap[pixelPosition]] = 2;
                    destinationPixel[pixelPosition] = 2;
                }
            }
        }
    }

    // Resolve all label equivalences to unify shapes under a single label.
    for (uint32_t i = 1; i < currentLabelID; ++i)
    {
        uint32_t labelRoot = i;
        while (labelEquivalence[labelRoot] != labelRoot)
        {
            labelRoot = labelEquivalence[labelRoot];
        }
        labelEquivalence[i] = labelRoot;

        // Make sure border connectivity is properly propagated so any label
        // ultimately connected to label 2 also becomes label 2.
        if (labelEquivalence[labelRoot] == 2)
        {
            labelEquivalence[i] = 2;
        }
    }

    // At the end, any shape with a border label is turned into background (0).
    // Shapes not touching the border remain as 1.
    // Second pass: Convert labels to final binary image
    // - Pixels connected to border (label 2) become background (0)
    // - All other object pixels become foreground (1)
    for (uint32_t y = 0; y < height; ++y)
    {
        for (uint32_t x = 0; x < width; ++x)
        {
            uint32_t pixelPosition = y * width + x;
            uint32_t label = labelMap[pixelPosition];

            if (labelEquivalence[label] == 2)
            {
                destinationPixel[pixelPosition] = 0;
            }
            else if (label != 0)
            {
                destinationPixel[pixelPosition] = 1;
            }
        }
    }

    // Clean up allocated memory
    free(labelEquivalence);
    free(labelMap);

    return 1; // This means we succeeded.
}

/*!
 * \brief Defines a unique compressed geometrical representation of an object.
 *
 * Binary skeleton is defined as the union of the set of pixels computed from
 * the difference of the n_th eroded image and the opening of the n_th
 * eroded image.
 * The number of erosions n required by the skeleton algorithm is the number
 * of erosions of the original image A by the structuring function B that
 * yields the null image.
 * The function does not necessarily produce a fully connected object.
 *
 * \param[in]  src  A pointer to the source image
 * \param[out] dst  A pointer to the destination image
 * \param[in]  mask A pointer to a square mask of size \p n
 * \param[in]  n    The size of the mask
 */
void skeleton(const image_t *src, image_t *dst, const uint8_t *mask, const uint8_t n)
{
    // Verify image validity
    ASSERT(src == NULL, "src image is invalid");
    ASSERT(dst == NULL, "dst image is invalid");
    ASSERT(src->data == NULL, "src data is invalid");
    ASSERT(dst->data == NULL, "dst data is invalid");
    ASSERT(src->type != IMGTYPE_UINT8, "src type is invalid");
    ASSERT(dst->type != IMGTYPE_UINT8, "dst type is invalid");

    // Verify mask validity
    ASSERT(mask == NULL, "mask is invalid");
    ASSERT(n == 0, "mask size can not be equal to 0");

    // Verify image consistency
    ASSERT(src == dst, "src and dst are the same images");
    ASSERT(src->cols != dst->cols, "src and dst have different number of columns");
    ASSERT(src->rows != dst->rows, "src and dst have different number of rows");

    clearUint8Image(dst);

    // Create temporary images
    image_t *org    = newUint8Image(src->cols, src->rows);
    image_t *eroded = newUint8Image(src->cols, src->rows);
    image_t *opened = newUint8Image(src->cols, src->rows);

    ASSERT(opened == NULL, "not enough memory for image allocation");

    copyUint8Image(src,org);

    // Loop as long as the original image has not been fully eroded
    uint8_t changes=1;
    while(changes)
    {
        changes = 0;

        // eroded = erode_n(org)
        erosion(org, eroded, mask, n);

        // Opening: an erosion followed by dilation
        // opened = open(erode_n(org), mask)
        dilation(eroded, opened, mask, n);

        // Loop all pixels, skip the border
        for(int32_t y=n/2; y<org->rows-(n/2); y++)
        {
            for(int32_t x=(n/2); x<org->cols-(n/2); x++)
            {
                // Calculate Kn(A): erode_n-1(org) - open(erode_n(org), mask)
                uint8_pixel_t p = getUint8Pixel(org,x,y) - getUint8Pixel(opened,x,y);

                // Create skeleton by the union of Kn(A) of all erosions
                setUint8Pixel(dst,x,y, getUint8Pixel(dst,x,y) | p);

                // Copy eroded image to original image
                setUint8Pixel(org,x,y, getUint8Pixel(eroded,x,y));

                // Continue as long as the original image has not yet been fully eroded
                if(getUint8Pixel(org,x,y) == 1)
                    changes=1;
            }
        }
    }

    // Cleanup
    deleteUint8Image(org);
    deleteUint8Image(eroded);
    deleteUint8Image(opened);
}
