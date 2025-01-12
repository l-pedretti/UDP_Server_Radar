/*
* DEEPCRAFT Studio 5.2.2135+b54b28f236fe7c6fb520f00b8de0ba1419f4422f
* Copyright © 2023- Imagimob AB, All Rights Reserved.
* 
* Generated at 12/16/2024 13:48:51 UTC. Any changes will be lost.
* 
* Model ID  eb829dac-b22b-433c-a809-25dcf8024af9
* 
* Memory    Size                      Efficiency
* Buffers   585728 bytes (RAM)        90 %
* State     61648 bytes (RAM)         100 %
* Readonly  75212 bytes (Flash)       100 %
* 
* Backend              tensorflow
* Keras Version        2.15.0
* Backend Model Type   Sequential
* Backend Model Name   conv2d-medium-balanced-1
* 
* Class Index | Symbol Label
* 0           | unlabelled
* 1           | fall
* 2           | lie
* 
* Layer                          Shape           Type       Function
* Sliding Window (data points)   [15,1024]       float      dequeue
*    window_shape = [15,1024]
*    stride = 1024
*    buffer_multiplier = 1
* Contextual Window (Sliding Window) [15,1024]       float      dequeue
*    contextual_length_sec = 3
*    prediction_freq = 5
* Input Layer                    [15,1024]       float      dequeue
*    shape = [15,1024]
* Reshape                        [15,1024,1]     float      dequeue
*    shape = [15,1024,1]
*    trainable = True
* Convolution 2D                 [8,512,16]      float      dequeue
*    filters = 16
*    kernel_size = [3,3]
*    strides = [2,2]
*    padding = same
*    activation = linear
*    use_bias = False
*    trainable = True
*    weight = float[3,3,1,16]
* Batch Normalization            [8,512,16]      float      dequeue
*    epsilon = 0.001
*    trainable = True
*    scale = True
*    center = True
*    axis = 3
*    gamma = float[16]
*    beta = float[16]
*    mean = float[16]
*    variance = float[16]
* Activation                     [8,512,16]      float      dequeue
*    activation = relu
*    trainable = True
* Convolution 2D                 [8,512,16]      float      dequeue
*    filters = 16
*    kernel_size = [3,3]
*    strides = [1,1]
*    padding = same
*    activation = linear
*    use_bias = False
*    trainable = True
*    weight = float[3,3,16,16]
* Convolution 2D                 [8,512,16]      float      dequeue
*    filters = 16
*    kernel_size = [3,3]
*    strides = [1,1]
*    padding = same
*    activation = linear
*    use_bias = False
*    trainable = True
*    weight = float[3,3,16,16]
* Batch Normalization            [8,512,16]      float      dequeue
*    epsilon = 0.001
*    trainable = True
*    scale = True
*    center = True
*    axis = 3
*    gamma = float[16]
*    beta = float[16]
*    mean = float[16]
*    variance = float[16]
* Activation                     [8,512,16]      float      dequeue
*    activation = relu
*    trainable = True
* Max pooling 2D                 [4,256,16]      float      dequeue
*    pool_size = [2,2]
*    strides = [2,2]
*    padding = valid
*    trainable = True
* Convolution 2D                 [4,256,32]      float      dequeue
*    filters = 32
*    kernel_size = [3,3]
*    strides = [1,1]
*    padding = same
*    activation = linear
*    use_bias = False
*    trainable = True
*    weight = float[3,3,16,32]
* Convolution 2D                 [4,256,32]      float      dequeue
*    filters = 32
*    kernel_size = [3,3]
*    strides = [1,1]
*    padding = same
*    activation = linear
*    use_bias = False
*    trainable = True
*    weight = float[3,3,32,32]
* Batch Normalization            [4,256,32]      float      dequeue
*    epsilon = 0.001
*    trainable = True
*    scale = True
*    center = True
*    axis = 3
*    gamma = float[32]
*    beta = float[32]
*    mean = float[32]
*    variance = float[32]
* Activation                     [4,256,32]      float      dequeue
*    activation = relu
*    trainable = True
* Global average pooling 2D      [32]            float      dequeue
* Dense                          [3]             float      dequeue
*    units = 3
*    use_bias = True
*    activation = linear
*    trainable = True
*    weight = float[32,3]
*    bias = float[3]
* Activation                     [3]             float      dequeue
*    activation = softmax
*    trainable = True
* 
* Exported functions:
* 
* int IMAI_dequeue(float *restrict data_out)
*    Description: Dequeue features. RET_SUCCESS (0) on success, RET_NODATA (-1) if no data is available, RET_NOMEM (-2) on internal memory error
*    Parameter data_out is Output of size float[3].
* 
* int IMAI_enqueue(const float *restrict data_in)
*    Description: Enqueue features. Returns SUCCESS (0) on success, else RET_NOMEM (-2) when low on memory.
*    Parameter data_in is Input of size float[1024].
* 
* void IMAI_init(void)
*    Description: Initializes buffers to initial state. This function also works as a reset function.
* 
* 
* Disclaimer:
*   The generated code relies on the optimizations done by the C compiler.
*   For example many for-loops of length 1 must be removed by the optimizer.
*   This can only be done if the functions are inlined and simplified.
*   Check disassembly if unsure.
*   tl;dr Compile using gcc with -O3 or -Ofast
*/

#ifndef _IMAI_MODEL_H_
#define _IMAI_MODEL_H_
#ifdef _MSC_VER
#pragma once
#endif

#include <stdint.h>

typedef struct {    
    char *name;
    double TP; // True Positive or Correct Positive Prediction
    double FN; // False Negative or Incorrect Negative Prediction
    double FP; // False Positive or Incorrect Positive Prediction
    double TN; // True Negative or Correct Negative Prediction
    double TPR; // True Positive Rate or Sensitivity, Recall
    double TNR; // True Negative Rate or Specificity, Selectivity
    double PPV; // Positive Predictive Value or Precision
    double NPV; // Negative Predictive Value
    double FNR; // False Negative Rate or Miss Rate
    double FPR; // False Positive Rate or Fall-Out
    double FDR; // False Discovery Rate
    double FOR; // False Omission Rate
    double F1S; // F1 Score
} IMAI_stats;

/*
* Tensorflow Test Set
* 
* (ACC) Accuracy 90.209 %
* (F1S) F1 Score 90.137 %
* 
* Name of class                                            (unlabelled)             fall              lie
* (TP) True Positive or Correct Positive Prediction                   0             1242             1301
* (FN) False Negative or Incorrect Negative Prediction                4              220               52
* (FP) False Positive or Incorrect Positive Prediction                0               52              224
* (TN) True Negative or Correct Negative Prediction                2815             1305             1242
* (TPR) True Positive Rate or Sensitivity, Recall                0.00 %          84.95 %          96.16 %
* (TNR) True Negative Rate or Specificity, Selectivity         100.00 %          96.17 %          84.72 %
* (PPV) Positive Predictive Value or Precision                 100.00 %          95.98 %          85.31 %
* (NPV) Negative Predictive Value                               99.86 %          85.57 %          95.98 %
* (FNR) False Negative Rate or Miss Rate                       100.00 %          15.05 %           3.84 %
* (FPR) False Positive Rate or Fall-Out                          0.00 %           3.83 %          15.28 %
* (FDR) False Discovery Rate                                   100.00 %           4.02 %          14.69 %
* (FOR) False Omission Rate                                      0.14 %          14.43 %           4.02 %
* (F1S) F1 Score                                                 0.00 %          90.13 %          90.41 %
*/


#define IMAI_TEST_AVG_ACC 0.9020929407591345 // Accuracy
#define IMAI_TEST_AVG_F1S 0.901368257630952 // F1 Score

#define IMAI_TEST_STATS { \
 {name: "(unlabelled)", TP: 0, FN: 4, FP: 0, TN: 2815, TPR: 0, TNR: 1, PPV: 1, NPV: 0.9985810571124, FNR: 1, FPR: 0, FDR: 1, FOR: 0.0014189428875, F1S: 0, }, \
 {name: "fall", TP: 1242, FN: 220, FP: 52, TN: 1305, TPR: 0.8495212038303, TNR: 0.9616801768607, PPV: 0.9598145285935, NPV: 0.8557377049180, FNR: 0.1504787961696, FPR: 0.0383198231392, FDR: 0.0401854714064, FOR: 0.1442622950819, F1S: 0.9013062409288, }, \
 {name: "lie", TP: 1301, FN: 52, FP: 224, TN: 1242, TPR: 0.9615668883961, TNR: 0.8472032742155, PPV: 0.8531147540983, NPV: 0.9598145285935, FNR: 0.0384331116038, FPR: 0.1527967257844, FDR: 0.1468852459016, FOR: 0.0401854714064, F1S: 0.9041000694927, }, \
}

#ifdef IMAI_STATS_ENABLED
static const IMAI_stats IMAI_test_stats[] = IMAI_TEST_STATS;
#endif

/*
* Tensorflow Train Set
* 
* (ACC) Accuracy 95.826 %
* (F1S) F1 Score 95.806 %
* 
* Name of class                                            (unlabelled)             fall              lie
* (TP) True Positive or Correct Positive Prediction                   0             3663             4234
* (FN) False Negative or Incorrect Negative Prediction                2              288               54
* (FP) False Positive or Incorrect Positive Prediction                0               56              288
* (TN) True Negative or Correct Negative Prediction                8239             4234             3665
* (TPR) True Positive Rate or Sensitivity, Recall                0.00 %          92.71 %          98.74 %
* (TNR) True Negative Rate or Specificity, Selectivity         100.00 %          98.69 %          92.71 %
* (PPV) Positive Predictive Value or Precision                 100.00 %          98.49 %          93.63 %
* (NPV) Negative Predictive Value                               99.98 %          93.63 %          98.55 %
* (FNR) False Negative Rate or Miss Rate                       100.00 %           7.29 %           1.26 %
* (FPR) False Positive Rate or Fall-Out                          0.00 %           1.31 %           7.29 %
* (FDR) False Discovery Rate                                   100.00 %           1.51 %           6.37 %
* (FOR) False Omission Rate                                      0.02 %           6.37 %           1.45 %
* (F1S) F1 Score                                                 0.00 %          95.51 %          96.12 %
*/


#define IMAI_TRAIN_AVG_ACC 0.9582574930226915 // Accuracy
#define IMAI_TRAIN_AVG_F1S 0.9580559733843641 // F1 Score

#define IMAI_TRAIN_STATS { \
 {name: "(unlabelled)", TP: 0, FN: 2, FP: 0, TN: 8239, TPR: 0, TNR: 1, PPV: 1, NPV: 0.9997573110059, FNR: 1, FPR: 0, FDR: 1, FOR: 0.0002426889940, F1S: 0, }, \
 {name: "fall", TP: 3663, FN: 288, FP: 56, TN: 4234, TPR: 0.9271070615034, TNR: 0.9869463869463, PPV: 0.9849421887604, NPV: 0.9363113666519, FNR: 0.0728929384965, FPR: 0.0130536130536, FDR: 0.0150578112395, FOR: 0.0636886333480, F1S: 0.9551499348109, }, \
 {name: "lie", TP: 4234, FN: 54, FP: 288, TN: 3665, TPR: 0.9874067164179, TNR: 0.9271439413103, PPV: 0.9363113666519, NPV: 0.9854799677332, FNR: 0.0125932835820, FPR: 0.0728560586896, FDR: 0.0636886333480, FOR: 0.0145200322667, F1S: 0.9611804767309, }, \
}

#ifdef IMAI_STATS_ENABLED
static const IMAI_stats IMAI_train_stats[] = IMAI_TRAIN_STATS;
#endif

/*
* Tensorflow Validation Set
* 
* (ACC) Accuracy 89.066 %
* (F1S) F1 Score 88.942 %
* 
* Name of class                                            (unlabelled)             fall              lie
* (TP) True Positive or Correct Positive Prediction                   0             1046             1414
* (FN) False Negative or Incorrect Negative Prediction                1              252               49
* (FP) False Positive or Incorrect Positive Prediction                0               49              253
* (TN) True Negative or Correct Negative Prediction                2761             1415             1046
* (TPR) True Positive Rate or Sensitivity, Recall                0.00 %          80.59 %          96.65 %
* (TNR) True Negative Rate or Specificity, Selectivity         100.00 %          96.65 %          80.52 %
* (PPV) Positive Predictive Value or Precision                 100.00 %          95.53 %          84.82 %
* (NPV) Negative Predictive Value                               99.96 %          84.88 %          95.53 %
* (FNR) False Negative Rate or Miss Rate                       100.00 %          19.41 %           3.35 %
* (FPR) False Positive Rate or Fall-Out                          0.00 %           3.35 %          19.48 %
* (FDR) False Discovery Rate                                   100.00 %           4.47 %          15.18 %
* (FOR) False Omission Rate                                      0.04 %          15.12 %           4.47 %
* (F1S) F1 Score                                                 0.00 %          87.42 %          90.35 %
*/


#define IMAI_VALIDATION_AVG_ACC 0.8906589427950761 // Accuracy
#define IMAI_VALIDATION_AVG_F1S 0.8894187200261501 // F1 Score

#define IMAI_VALIDATION_STATS { \
 {name: "(unlabelled)", TP: 0, FN: 1, FP: 0, TN: 2761, TPR: 0, TNR: 1, PPV: 1, NPV: 0.9996379435191, FNR: 1, FPR: 0, FDR: 1, FOR: 0.0003620564808, F1S: 0, }, \
 {name: "fall", TP: 1046, FN: 252, FP: 49, TN: 1415, TPR: 0.8058551617873, TNR: 0.9665300546448, PPV: 0.9552511415525, NPV: 0.8488302339532, FNR: 0.1941448382126, FPR: 0.0334699453551, FDR: 0.0447488584474, FOR: 0.1511697660467, F1S: 0.8742164646886, }, \
 {name: "lie", TP: 1414, FN: 49, FP: 253, TN: 1046, TPR: 0.9665071770334, TNR: 0.8052347959969, PPV: 0.8482303539292, NPV: 0.9552511415525, FNR: 0.0334928229665, FPR: 0.1947652040030, FDR: 0.1517696460707, FOR: 0.0447488584474, F1S: 0.9035143769968, }, \
}

#ifdef IMAI_STATS_ENABLED
static const IMAI_stats IMAI_validation_stats[] = IMAI_VALIDATION_STATS;
#endif

#define IMAI_API_QUEUE

// All symbols in order
#define IMAI_SYMBOL_MAP {"unlabelled", "fall", "lie"}

// Model GUID (16 bytes)
#define IMAI_MODEL_ID {0xac, 0x9d, 0x82, 0xeb, 0x2b, 0xb2, 0x3c, 0x43, 0xa8, 0x09, 0x25, 0xdc, 0xf8, 0x02, 0x4a, 0xf9}

// First nibble is bit encoding, second nibble is number of bytes
#define IMAGINET_TYPES_NONE	(0x0)
#define IMAGINET_TYPES_FLOAT32	(0x14)
#define IMAGINET_TYPES_FLOAT64	(0x18)
#define IMAGINET_TYPES_INT8	(0x21)
#define IMAGINET_TYPES_INT16	(0x22)
#define IMAGINET_TYPES_INT32	(0x24)
#define IMAGINET_TYPES_INT64	(0x28)
#define IMAGINET_TYPES_QDYN8	(0x31)
#define IMAGINET_TYPES_QDYN16	(0x32)
#define IMAGINET_TYPES_QDYN32	(0x34)

// data_in [1024] (4096 bytes)
#define IMAI_DATA_IN_COUNT (1024)
#define IMAI_DATA_IN_TYPE float
#define IMAI_DATA_IN_TYPE_ID IMAGINET_TYPES_FLOAT32
#define IMAI_DATA_IN_SCALE (1)
#define IMAI_DATA_IN_OFFSET (0)
#define IMAI_DATA_IN_IS_QUANTIZED (0)

// data_out [3] (12 bytes)
#define IMAI_DATA_OUT_COUNT (3)
#define IMAI_DATA_OUT_TYPE float
#define IMAI_DATA_OUT_TYPE_ID IMAGINET_TYPES_FLOAT32
#define IMAI_DATA_OUT_SCALE (1)
#define IMAI_DATA_OUT_OFFSET (0)
#define IMAI_DATA_OUT_IS_QUANTIZED (0)

#define IMAI_KEY_MAX (36)



// Return codes
#define IMAI_RET_SUCCESS 0
#define IMAI_RET_NODATA -1
#define IMAI_RET_NOMEM -2

// Exported methods
int IMAI_dequeue(float *restrict data_out);
int IMAI_enqueue(const float *restrict data_in);
void IMAI_init(void);

#endif /* _IMAI_MODEL_H_ */
