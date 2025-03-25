// coding_and_compression.h
#ifndef CODING_AND_COMPRESSION_H
#define CODING_AND_COMPRESSION_H

#include "image.h"
#include <stdint.h>

// Define a structure for Huffman tree nodes
typedef struct HuffmanNode {
    uint8_t pixel;             // Pixel value (0-255)
    uint32_t frequency;        // Frequency of the pixel
    struct HuffmanNode* left;  // Left child
    struct HuffmanNode* right; // Right child
} HuffmanNode;

// Define a structure for Huffman codes
typedef struct {
    uint32_t code;  // The bit pattern (right-aligned)
    uint8_t length; // Number of bits in the code
} HuffmanCode;

// Define a structure to hold encoded data
typedef struct {
    uint8_t* data;         // Encoded bit stream
    uint32_t size;         // Size in bytes
    uint32_t width;        // Original image width
    uint32_t height;       // Original image height
    HuffmanCode codes[256]; // Huffman codes for each pixel value
} EncodedImage;

/**
 * @brief Encodes an image using Huffman coding
 * 
 * @param src Pointer to the source image (must be IMGTYPE_UINT8)
 * @param encoded Pointer to the structure that will hold the encoded data
 * @return 1 on success, 0 on failure
 */
int32_t huffmanEncode(const image_t* src, EncodedImage* encoded);

/**
 * @brief Decodes an image using Huffman coding
 * 
 * @param encoded Pointer to the structure containing the encoded data
 * @param dst Pointer to the destination image (must be IMGTYPE_UINT8)
 * @return 1 on success, 0 on failure
 */
int32_t huffmanDecode(const EncodedImage* encoded, image_t* dst);

/**
 * @brief Frees memory allocated for encoded image
 * 
 * @param encoded Pointer to the encoded image structure
 */
void freeEncodedImage(EncodedImage* encoded);

#endif // CODING_AND_COMPRESSION_H