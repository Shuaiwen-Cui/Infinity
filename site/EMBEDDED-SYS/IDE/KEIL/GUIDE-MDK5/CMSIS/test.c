#include "arm_math.h" // ARM::CMSIS:DSP
const float32_t buf_A[9] = {
    // Matrix A buffer and values
    1.0,
    32.0,
    4.0,
    1.0,
    32.0,
    64.0,
    1.0,
    16.0,
    4.0,
};
float32_t buf_AT[9];          // Buffer for A Transpose (AT)
float32_t buf_ATmA[9];        // Buffer for (AT * A)
arm_matrix_instance_f32 A;    // Matrix A
arm_matrix_instance_f32 AT;   // Matrix AT(A transpose)
arm_matrix_instance_f32 ATmA; // Matrix ATmA( AT multiplied by A)
uint32_t rows = 3;            // Matrix rows
uint32_t cols = 3;            // Matrix columns
int main(void)
{
    // Initialize all matrixes with rows, columns, and data array
    arm_mat_init_f32(&A, rows, cols, (float32_t *)buf_A); // Matrix A
    arm_mat_init_f32(&AT, rows, cols, buf_AT);            // Matrix AT
    arm_mat_init_f32(&ATmA, rows, cols, buf_ATmA);        // Matrix ATmA
    arm_mat_trans_f32(&A, &AT);                           // Calculate A Transpose (AT)
    arm_mat_mult_f32(&AT, &A, &ATmA);                     // Multiply AT with A
    while (1)
        ;
}