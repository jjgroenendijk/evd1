/**
 * @file coding_and_compression.c
 * @brief Implementation of image coding and compression algorithms
 * 
 * This file provides implementations of lossless image compression algorithms,
 * specifically Huffman coding. Huffman coding is a variable-length encoding method
 * that assigns shorter codes to more frequent pixel values, resulting in overall
 * data compression. This implementation supports both encoding (compression) and
 * decoding (decompression) operations for grayscale images.
 * 
 * The Huffman algorithm works by:
 * 1. Calculating frequency of each pixel value in the image
 * 2. Building a binary tree where leaf nodes represent pixel values
 * 3. Assigning variable-length bit codes to each leaf node
 * 4. Encoding the image using these codes
 * 
 * For decoding, the process is reversed using the stored Huffman codes.
 */
#include "coding_and_compression.h"
#include <stdlib.h>
#include <string.h>

/**
 * @brief Priority queue implementation for Huffman tree construction
 * 
 * This structure implements a min heap to efficiently extract the nodes
 * with lowest frequency during Huffman tree construction.
 */
typedef struct {
    HuffmanNode** array;  /**< Array of pointers to HuffmanNode elements */
    int32_t size;         /**< Current number of elements in the queue */
    int32_t capacity;     /**< Maximum capacity of the queue */
} PriorityQueue;

/* Function prototypes for helper functions */
static PriorityQueue* createPriorityQueue(int32_t capacity);
static void freePriorityQueue(PriorityQueue* queue);
static void swap(HuffmanNode** a, HuffmanNode** b);
static void heapify(PriorityQueue* queue, int32_t idx);
static void insert(PriorityQueue* queue, HuffmanNode* node);
static HuffmanNode* extractMin(PriorityQueue* queue);
static HuffmanNode* createNode(uint8_t pixel, uint32_t freq);
static void freeNode(HuffmanNode* node);
static void generateCodes(HuffmanNode* root, uint32_t code, uint8_t depth, HuffmanCode codes[256]);
static void writeBit(uint8_t* buffer, uint32_t* pos, uint8_t bit);
static uint8_t readBit(const uint8_t* buffer, uint32_t pos);

/**
 * @brief Create a new priority queue
 * 
 * Creates and initializes a priority queue with the specified capacity.
 * The priority queue is implemented as a min heap.
 * 
 * @param capacity Maximum number of elements the queue can hold
 * @return Pointer to the created priority queue, or NULL if memory allocation fails
 */
static PriorityQueue* createPriorityQueue(int32_t capacity) {
    PriorityQueue* queue = (PriorityQueue*)malloc(sizeof(PriorityQueue));
    if (!queue) return NULL;
    
    queue->array = (HuffmanNode**)malloc(capacity * sizeof(HuffmanNode*));
    if (!queue->array) {
        free(queue);
        return NULL;
    }
    
    queue->size = 0;
    queue->capacity = capacity;
    return queue;
}

/**
 * @brief Free resources associated with a priority queue
 * 
 * Deallocates memory used by the priority queue structure and its array.
 * Note that this does not free the HuffmanNode elements stored in the queue.
 * 
 * @param queue Pointer to the priority queue to free
 */
static void freePriorityQueue(PriorityQueue* queue) {
    if (!queue) return;
    free(queue->array);
    free(queue);
}

/**
 * @brief Swap two HuffmanNode pointers
 * 
 * Helper function used in priority queue operations to swap nodes.
 * 
 * @param a Pointer to the first HuffmanNode pointer
 * @param b Pointer to the second HuffmanNode pointer
 */
static void swap(HuffmanNode** a, HuffmanNode** b) {
    HuffmanNode* temp = *a;
    *a = *b;
    *b = temp;
}

/**
 * @brief Maintain min heap property at a given index
 * 
 * Recursively adjusts the heap to ensure the min heap property is maintained,
 * where the parent node has a frequency value less than or equal to its children.
 * 
 * @param queue Pointer to the priority queue
 * @param idx Index at which to start heapifying
 */
static void heapify(PriorityQueue* queue, int32_t idx) {
    int32_t smallest = idx;
    int32_t left = 2 * idx + 1;
    int32_t right = 2 * idx + 2;
    
    // Find the smallest among the current node and its children
    if (left < queue->size && 
        queue->array[left]->frequency < queue->array[smallest]->frequency)
        smallest = left;
    
    if (right < queue->size && 
        queue->array[right]->frequency < queue->array[smallest]->frequency)
        smallest = right;
    
    // If the smallest is not the current node, swap and continue heapifying
    if (smallest != idx) {
        swap(&queue->array[idx], &queue->array[smallest]);
        heapify(queue, smallest);
    }
}

/**
 * @brief Insert a node into the priority queue
 * 
 * Adds a new HuffmanNode to the priority queue and maintains the min heap property.
 * If the queue is already at capacity, the insertion is ignored.
 * 
 * @param queue Pointer to the priority queue
 * @param node The HuffmanNode to insert
 */
static void insert(PriorityQueue* queue, HuffmanNode* node) {
    if (queue->size == queue->capacity) return;
    
    // Insert at the end
    int32_t i = queue->size;
    queue->array[i] = node;
    queue->size++;
    
    // Fix the min heap property (bubble up if needed)
    while (i > 0 && queue->array[i]->frequency < 
           queue->array[(i - 1) / 2]->frequency) {
        swap(&queue->array[i], &queue->array[(i - 1) / 2]);
        i = (i - 1) / 2;
    }
}

/**
 * @brief Extract the minimum frequency node from the priority queue
 * 
 * Removes and returns the node with the minimum frequency from the queue,
 * then rearranges the queue to maintain the min heap property.
 * 
 * @param queue Pointer to the priority queue
 * @return The HuffmanNode with minimum frequency, or NULL if the queue is empty
 */
static HuffmanNode* extractMin(PriorityQueue* queue) {
    if (queue->size <= 0) return NULL;
    
    // Store the minimum value
    HuffmanNode* root = queue->array[0];
    
    // Replace the root with the last element
    queue->array[0] = queue->array[queue->size - 1];
    queue->size--;
    
    // Restore the min heap property
    heapify(queue, 0);
    
    return root;
}

/**
 * @brief Create a new Huffman tree node
 * 
 * Allocates and initializes a new HuffmanNode with the specified pixel value and frequency.
 * 
 * @param pixel The pixel value (0-255) represented by this node
 * @param freq The frequency of occurrence of this pixel in the image
 * @return Pointer to the created node, or NULL if memory allocation fails
 */
static HuffmanNode* createNode(uint8_t pixel, uint32_t freq) {
    HuffmanNode* node = (HuffmanNode*)malloc(sizeof(HuffmanNode));
    if (!node) return NULL;
    
    node->pixel = pixel;
    node->frequency = freq;
    node->left = node->right = NULL;
    
    return node;
}

/**
 * @brief Free a Huffman tree node and all its children
 * 
 * Recursively deallocates memory for a Huffman tree node and all its descendants.
 * 
 * @param node Pointer to the HuffmanNode to free
 */
static void freeNode(HuffmanNode* node) {
    if (!node) return;
    freeNode(node->left);
    freeNode(node->right);
    free(node);
}

/**
 * @brief Generate Huffman codes for each pixel value
 * 
 * Traverses the Huffman tree to generate binary codes for each leaf node (pixel value).
 * For each left branch, a 0 is appended to the code, and for each right branch, a 1 is appended.
 * 
 * @param root Root of the Huffman tree
 * @param code Current code value (starts at 0)
 * @param depth Current depth in the tree (code length)
 * @param codes Array to store the generated codes for each pixel value
 */
static void generateCodes(HuffmanNode* root, uint32_t code, uint8_t depth, HuffmanCode codes[256]) {
    if (!root) return;
    
    // Found a leaf node (an actual pixel value)
    if (!root->left && !root->right) {
        codes[root->pixel].code = code;
        codes[root->pixel].length = depth;
        return;
    }
    
    // Traverse left (append 0 to the code)
    generateCodes(root->left, code << 1, depth + 1, codes);
    
    // Traverse right (append 1 to the code)
    generateCodes(root->right, (code << 1) | 1, depth + 1, codes);
}

/**
 * @brief Write a single bit to a byte buffer
 * 
 * Sets or clears a bit at the specified position in a byte buffer.
 * Bits are written in MSB-first order (most significant bit first).
 * 
 * @param buffer Byte buffer to write to
 * @param pos Pointer to the current bit position (will be incremented)
 * @param bit Bit value to write (0 or 1)
 */
static void writeBit(uint8_t* buffer, uint32_t* pos, uint8_t bit) {
    uint32_t bytePos = *pos / 8;  // Byte index in the buffer
    uint8_t bitPos = *pos % 8;    // Bit position within the byte (0-7)
    
    // Clear or set the bit
    if (bit) {
        buffer[bytePos] |= (1 << (7 - bitPos));  // Set the bit
    } else {
        buffer[bytePos] &= ~(1 << (7 - bitPos)); // Clear the bit
    }
    
    (*pos)++;  // Move to the next bit position
}

/**
 * @brief Read a single bit from a byte buffer
 * 
 * Reads the bit at the specified position from a byte buffer.
 * Bits are read in MSB-first order (most significant bit first).
 * 
 * @param buffer Byte buffer to read from
 * @param pos Bit position to read
 * @return The bit value (0 or 1)
 */
static uint8_t readBit(const uint8_t* buffer, uint32_t pos) {
    uint32_t bytePos = pos / 8;  // Byte index in the buffer
    uint8_t bitPos = pos % 8;    // Bit position within the byte (0-7)
    
    return (buffer[bytePos] >> (7 - bitPos)) & 1;
}

/**
 * @brief Encode an image using Huffman coding
 * 
 * Compresses a grayscale image using the Huffman coding algorithm. The function:
 * 1. Computes pixel frequency distribution
 * 2. Builds a Huffman tree based on these frequencies
 * 3. Generates variable-length codes for each pixel value
 * 4. Encodes the image using these codes
 * 
 * @param src Pointer to the source image (must be of type IMGTYPE_UINT8)
 * @param encoded Pointer to an EncodedImage structure that will store the compressed data
 * @return 1 on success, 0 on failure (memory allocation error)
 */
int32_t huffmanEncode(const image_t* src, EncodedImage* encoded) {

    // Count frequency of each pixel value in the image
    uint32_t frequency[256] = {0};
    uint8_t* pixelData = (uint8_t*)src->data;
    uint32_t totalPixels = src->cols * src->rows;
    
    for (uint32_t i = 0; i < totalPixels; i++) {
        frequency[pixelData[i]]++;
    }
    
    // Create leaf nodes for each pixel value with non-zero frequency
    PriorityQueue* queue = createPriorityQueue(256);
    if (!queue) return 0;
    
    for (int32_t i = 0; i < 256; i++) {
        if (frequency[i] > 0) {
            HuffmanNode* node = createNode(i, frequency[i]);
            if (!node) {
                freePriorityQueue(queue);
                return 0;
            }
            insert(queue, node);
        }
    }
    
    // Special case: if there's only one unique pixel value in the image
    if (queue->size == 1) {
        HuffmanNode* node = queue->array[0];
        encoded->codes[node->pixel].code = 0;
        encoded->codes[node->pixel].length = 1;
        
        // Allocate memory for encoded data (1 bit per pixel, rounded up to bytes)
        encoded->data = (uint8_t*)malloc(totalPixels / 8 + 1);
        if (!encoded->data) {
            freePriorityQueue(queue);
            return 0;
        }
        
        // Initialize all bits to 0
        memset(encoded->data, 0, totalPixels / 8 + 1);
        
        encoded->size = totalPixels / 8 + 1;
        encoded->width = src->cols;
        encoded->height = src->rows;
        
        freePriorityQueue(queue);
        return 1;
    }
    
    // Build Huffman tree by repeatedly merging the two nodes with lowest frequencies
    HuffmanNode* root = NULL;
    while (queue->size > 1) {
        // Extract the two nodes with lowest frequencies
        HuffmanNode* left = extractMin(queue);
        HuffmanNode* right = extractMin(queue);
        
        // Create new internal node with frequency = sum of children
        HuffmanNode* internal = createNode(0, left->frequency + right->frequency);
        if (!internal) {
            freeNode(left);
            freeNode(right);
            freePriorityQueue(queue);
            return 0;
        }
        
        // Set left and right children
        internal->left = left;
        internal->right = right;
        
        // Insert the new internal node back into the queue
        insert(queue, internal);
    }
    
    // The root of the Huffman tree is the last node in the queue
    root = extractMin(queue);
    
    // Generate Huffman codes for each pixel value by traversing the tree
    // Initialize all codes to 0
    for (int32_t i = 0; i < 256; i++) {
        encoded->codes[i].code = 0;
        encoded->codes[i].length = 0;
    }
    
    // Recursively generate codes for all leaf nodes
    generateCodes(root, 0, 0, encoded->codes);
    
    // Calculate the total size needed for the encoded data (rough estimate)
    uint32_t totalBits = 0;
    for (uint32_t i = 0; i < totalPixels; i++) {
        totalBits += encoded->codes[pixelData[i]].length;
    }
    
    // Allocate memory for encoded data
    encoded->data = (uint8_t*)malloc(totalBits / 8 + 1);
    if (!encoded->data) {
        freeNode(root);
        freePriorityQueue(queue);
        return 0;
    }
    
    // Initialize all bits to 0
    memset(encoded->data, 0, totalBits / 8 + 1);
    
    // Encode the image by replacing each pixel with its Huffman code
    uint32_t bitPos = 0;
    for (uint32_t i = 0; i < totalPixels; i++) {
        uint8_t pixel = pixelData[i];
        uint32_t code = encoded->codes[pixel].code;
        uint8_t codeLength = encoded->codes[pixel].length;
        
        // Write the code bits from MSB to LSB
        for (int32_t j = codeLength - 1; j >= 0; j--) {
            uint8_t bit = (code >> j) & 1;
            writeBit(encoded->data, &bitPos, bit);
        }
    }
    
    encoded->size = (bitPos + 7) / 8; // Round up to nearest byte
    encoded->width = src->cols;
    encoded->height = src->rows;
    
    // Clean up resources
    freeNode(root);
    freePriorityQueue(queue);
    
    return 1;
}

/**
 * @brief Decode a Huffman-encoded image
 * 
 * Decompresses a Huffman-encoded image back to its original form. The function:
 * 1. Rebuilds the Huffman tree from the stored codes
 * 2. Reads bits from the encoded data
 * 3. Traverses the tree according to these bits
 * 4. Outputs the pixel values from leaf nodes
 * 
 * @param encoded Pointer to the encoded image data and Huffman codes
 * @param dst Pointer to the destination image (must be of type IMGTYPE_UINT8)
 * @return 1 on success, 0 on failure (memory allocation error or invalid codes)
 */
int32_t huffmanDecode(const EncodedImage* encoded, image_t* dst) {
    
    // Rebuild the Huffman tree from the stored codes
    HuffmanNode* root = createNode(0, 0);
    if (!root) return 0;
    
    // Insert each code into the tree
    for (int32_t i = 0; i < 256; i++) {
        if (encoded->codes[i].length > 0) {
            uint32_t code = encoded->codes[i].code;
            uint8_t length = encoded->codes[i].length;
            
            HuffmanNode* current = root;
            
            // Navigate through the tree according to the code bits
            for (int32_t j = length - 1; j >= 0; j--) {
                uint8_t bit = (code >> j) & 1;
                
                if (bit == 0) {
                    // Create left child if it doesn't exist
                    if (!current->left) {
                        current->left = createNode(0, 0);
                        if (!current->left) {
                            freeNode(root);
                            return 0;
                        }
                    }
                    current = current->left;
                } else {
                    // Create right child if it doesn't exist
                    if (!current->right) {
                        current->right = createNode(0, 0);
                        if (!current->right) {
                            freeNode(root);
                            return 0;
                        }
                    }
                    current = current->right;
                }
                
                // If this is the last bit, mark the node with the pixel value
                if (j == 0) {
                    current->pixel = i;
                }
            }
        }
    }
    
    // Decode the image by traversing the tree for each encoded sequence
    uint8_t* pixelData = (uint8_t*)dst->data;
    uint32_t totalPixels = dst->cols * dst->rows;
    uint32_t bitPos = 0;
    
    for (uint32_t i = 0; i < totalPixels; i++) {
        HuffmanNode* current = root;
        
        // Navigate the tree according to the bits in the encoded data
        while (current->left || current->right) {
            uint8_t bit = readBit(encoded->data, bitPos++);
            current = bit ? current->right : current->left;
            
            // Handle invalid codes (should never happen with properly encoded data)
            if (!current) {
                freeNode(root);
                return 0;
            }
        }
        
        // Found a leaf node - output the pixel value
        pixelData[i] = current->pixel;
    }
    
    // Clean up resources
    freeNode(root);
    
    return 1;
}

/**
 * @brief Free resources associated with an encoded image
 * 
 * Deallocates memory used by the encoded image data.
 * 
 * @param encoded Pointer to the encoded image structure
 */
void freeEncodedImage(EncodedImage* encoded) {
    if (!encoded) return;
    if (encoded->data) free(encoded->data);
    encoded->data = NULL;
    encoded->size = 0;
}
