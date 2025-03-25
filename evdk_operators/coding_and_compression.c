// coding_and_compression.c
#include "coding_and_compression.h"
#include <stdlib.h>
#include <string.h>

// Helper structure for priority queue (min heap)
typedef struct {
    HuffmanNode** array;
    int32_t size;
    int32_t capacity;
} PriorityQueue;

// Function prototypes for helpers
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

// Create a priority queue
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

// Free a priority queue
static void freePriorityQueue(PriorityQueue* queue) {
    if (!queue) return;
    free(queue->array);
    free(queue);
}

// Swap two nodes
static void swap(HuffmanNode** a, HuffmanNode** b) {
    HuffmanNode* temp = *a;
    *a = *b;
    *b = temp;
}

// Heapify at a given index
static void heapify(PriorityQueue* queue, int32_t idx) {
    int32_t smallest = idx;
    int32_t left = 2 * idx + 1;
    int32_t right = 2 * idx + 2;
    
    if (left < queue->size && 
        queue->array[left]->frequency < queue->array[smallest]->frequency)
        smallest = left;
    
    if (right < queue->size && 
        queue->array[right]->frequency < queue->array[smallest]->frequency)
        smallest = right;
    
    if (smallest != idx) {
        swap(&queue->array[idx], &queue->array[smallest]);
        heapify(queue, smallest);
    }
}

// Insert a node into the priority queue
static void insert(PriorityQueue* queue, HuffmanNode* node) {
    if (queue->size == queue->capacity) return;
    
    // Insert at the end
    int32_t i = queue->size;
    queue->array[i] = node;
    queue->size++;
    
    // Fix the min heap property
    while (i > 0 && queue->array[i]->frequency < 
           queue->array[(i - 1) / 2]->frequency) {
        swap(&queue->array[i], &queue->array[(i - 1) / 2]);
        i = (i - 1) / 2;
    }
}

// Extract the minimum node from the priority queue
static HuffmanNode* extractMin(PriorityQueue* queue) {
    if (queue->size <= 0) return NULL;
    
    HuffmanNode* root = queue->array[0];
    queue->array[0] = queue->array[queue->size - 1];
    queue->size--;
    
    heapify(queue, 0);
    
    return root;
}

// Create a new node
static HuffmanNode* createNode(uint8_t pixel, uint32_t freq) {
    HuffmanNode* node = (HuffmanNode*)malloc(sizeof(HuffmanNode));
    if (!node) return NULL;
    
    node->pixel = pixel;
    node->frequency = freq;
    node->left = node->right = NULL;
    
    return node;
}

// Free a node and its children
static void freeNode(HuffmanNode* node) {
    if (!node) return;
    freeNode(node->left);
    freeNode(node->right);
    free(node);
}

// Generate Huffman codes by traversing the tree
static void generateCodes(HuffmanNode* root, uint32_t code, uint8_t depth, HuffmanCode codes[256]) {
    if (!root) return;
    
    // Found a leaf node
    if (!root->left && !root->right) {
        codes[root->pixel].code = code;
        codes[root->pixel].length = depth;
        return;
    }
    
    // Traverse left (append 0)
    generateCodes(root->left, code << 1, depth + 1, codes);
    
    // Traverse right (append 1)
    generateCodes(root->right, (code << 1) | 1, depth + 1, codes);
}

// Write a bit to a buffer
static void writeBit(uint8_t* buffer, uint32_t* pos, uint8_t bit) {
    uint32_t bytePos = *pos / 8;
    uint8_t bitPos = *pos % 8;
    
    // Clear or set the bit
    if (bit) {
        buffer[bytePos] |= (1 << (7 - bitPos));
    } else {
        buffer[bytePos] &= ~(1 << (7 - bitPos));
    }
    
    (*pos)++;
}

// Read a bit from a buffer
static uint8_t readBit(const uint8_t* buffer, uint32_t pos) {
    uint32_t bytePos = pos / 8;
    uint8_t bitPos = pos % 8;
    
    return (buffer[bytePos] >> (7 - bitPos)) & 1;
}

// Huffman encoding implementation
int32_t huffmanEncode(const image_t* src, EncodedImage* encoded) {

    // Count frequency of each pixel value
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
    
    // Special case: if there's only one unique pixel value
    if (queue->size == 1) {
        HuffmanNode* node = queue->array[0];
        encoded->codes[node->pixel].code = 0;
        encoded->codes[node->pixel].length = 1;
        
        // Allocate memory for encoded data
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
    
    // Build Huffman tree by merging nodes
    HuffmanNode* root = NULL;
    while (queue->size > 1) {
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
        
        internal->left = left;
        internal->right = right;
        
        insert(queue, internal);
    }
    
    // The root of the Huffman tree is the last node in the queue
    root = extractMin(queue);
    
    // Generate Huffman codes for each pixel value
    // Initialize all codes to 0
    for (int32_t i = 0; i < 256; i++) {
        encoded->codes[i].code = 0;
        encoded->codes[i].length = 0;
    }
    
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
    
    // Encode the image
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
    
    // Clean up
    freeNode(root);
    freePriorityQueue(queue);
    
    return 1;
}

// Huffman decoding implementation
int32_t huffmanDecode(const EncodedImage* encoded, image_t* dst) {
    
    // Rebuild the Huffman tree from the codes
    HuffmanNode* root = createNode(0, 0);
    if (!root) return 0;
    
    // Insert each code into the tree
    for (int32_t i = 0; i < 256; i++) {
        if (encoded->codes[i].length > 0) {
            uint32_t code = encoded->codes[i].code;
            uint8_t length = encoded->codes[i].length;
            
            HuffmanNode* current = root;
            
            // Navigate through the tree according to the code
            for (int32_t j = length - 1; j >= 0; j--) {
                uint8_t bit = (code >> j) & 1;
                
                if (bit == 0) {
                    if (!current->left) {
                        current->left = createNode(0, 0);
                        if (!current->left) {
                            freeNode(root);
                            return 0;
                        }
                    }
                    current = current->left;
                } else {
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
    
    // Decode the image
    uint8_t* pixelData = (uint8_t*)dst->data;
    uint32_t totalPixels = dst->cols * dst->rows;
    uint32_t bitPos = 0;
    
    for (uint32_t i = 0; i < totalPixels; i++) {
        HuffmanNode* current = root;
        
        // Navigate the tree according to the bits in the encoded data
        while (current->left || current->right) {
            uint8_t bit = readBit(encoded->data, bitPos++);
            current = bit ? current->right : current->left;
            
            // Handle invalid codes
            if (!current) {
                freeNode(root);
                return 0;
            }
        }
        
        // Found a leaf node
        pixelData[i] = current->pixel;
    }
    
    // Clean up
    freeNode(root);
    
    return 1;
}

// Free encoded image
void freeEncodedImage(EncodedImage* encoded) {
    if (!encoded) return;
    if (encoded->data) free(encoded->data);
    encoded->data = NULL;
    encoded->size = 0;
}