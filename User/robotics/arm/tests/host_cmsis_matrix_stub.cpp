#include <arm_math.h>

namespace {

bool same_shape(const arm_matrix_instance_f32* a,
                const arm_matrix_instance_f32* b) {
  return a != nullptr && b != nullptr && a->numRows == b->numRows &&
         a->numCols == b->numCols;
}

bool output_shape(const arm_matrix_instance_f32* src,
                  const arm_matrix_instance_f32* dst) {
  return same_shape(src, dst);
}

}  // namespace

extern "C" {

void arm_mat_init_f32(arm_matrix_instance_f32* s,
                      uint16_t rows,
                      uint16_t cols,
                      float32_t* data) {
  if (s == nullptr) {
    return;
  }
  s->numRows = rows;
  s->numCols = cols;
  s->pData = data;
}

arm_status arm_mat_add_f32(const arm_matrix_instance_f32* a,
                           const arm_matrix_instance_f32* b,
                           arm_matrix_instance_f32* dst) {
  if (!same_shape(a, b) || !output_shape(a, dst)) {
    return ARM_MATH_SIZE_MISMATCH;
  }
  const uint32_t count = static_cast<uint32_t>(a->numRows) * a->numCols;
  for (uint32_t i = 0; i < count; ++i) {
    dst->pData[i] = a->pData[i] + b->pData[i];
  }
  return ARM_MATH_SUCCESS;
}

arm_status arm_mat_sub_f32(const arm_matrix_instance_f32* a,
                           const arm_matrix_instance_f32* b,
                           arm_matrix_instance_f32* dst) {
  if (!same_shape(a, b) || !output_shape(a, dst)) {
    return ARM_MATH_SIZE_MISMATCH;
  }
  const uint32_t count = static_cast<uint32_t>(a->numRows) * a->numCols;
  for (uint32_t i = 0; i < count; ++i) {
    dst->pData[i] = a->pData[i] - b->pData[i];
  }
  return ARM_MATH_SUCCESS;
}

arm_status arm_mat_scale_f32(const arm_matrix_instance_f32* src,
                             float32_t scale,
                             arm_matrix_instance_f32* dst) {
  if (!output_shape(src, dst)) {
    return ARM_MATH_SIZE_MISMATCH;
  }
  const uint32_t count = static_cast<uint32_t>(src->numRows) * src->numCols;
  for (uint32_t i = 0; i < count; ++i) {
    dst->pData[i] = src->pData[i] * scale;
  }
  return ARM_MATH_SUCCESS;
}

arm_status arm_mat_trans_f32(const arm_matrix_instance_f32* src,
                             arm_matrix_instance_f32* dst) {
  if (src == nullptr || dst == nullptr ||
      src->numRows != dst->numCols ||
      src->numCols != dst->numRows) {
    return ARM_MATH_SIZE_MISMATCH;
  }
  for (uint16_t row = 0; row < src->numRows; ++row) {
    for (uint16_t col = 0; col < src->numCols; ++col) {
      dst->pData[col * dst->numCols + row] =
          src->pData[row * src->numCols + col];
    }
  }
  return ARM_MATH_SUCCESS;
}

arm_status arm_mat_mult_f32(const arm_matrix_instance_f32* a,
                            const arm_matrix_instance_f32* b,
                            arm_matrix_instance_f32* dst) {
  if (a == nullptr || b == nullptr || dst == nullptr ||
      a->numCols != b->numRows ||
      dst->numRows != a->numRows ||
      dst->numCols != b->numCols) {
    return ARM_MATH_SIZE_MISMATCH;
  }
  for (uint16_t row = 0; row < dst->numRows; ++row) {
    for (uint16_t col = 0; col < dst->numCols; ++col) {
      float32_t sum = 0.0f;
      for (uint16_t k = 0; k < a->numCols; ++k) {
        sum += a->pData[row * a->numCols + k] *
               b->pData[k * b->numCols + col];
      }
      dst->pData[row * dst->numCols + col] = sum;
    }
  }
  return ARM_MATH_SUCCESS;
}

}  // extern "C"
