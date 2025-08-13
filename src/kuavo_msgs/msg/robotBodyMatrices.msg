# robotBodyMatrices.msg
Header header

# 时间戳
time timestamp

# 矩阵数据 - 每个矩阵是4x4的齐次变换矩阵，展平为16个float64
float64[] matrices_data

# 矩阵数量
uint32 num_matrices

# 身体部位名称列表
string[] body_parts

# 数据来源标识
string data_source
