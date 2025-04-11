/*!
****************************************************************************************************
* 文件名称：bitconverter.h
* 功能简介：该文件是字节流转换模块的接口头文件
* 文件作者：Haotian
* 创建日期：2020-09-25
* 版权声明：All Rights Reserved.
****************************************************************************************************
*/
#ifndef INCLUDE_HQHP_BITCONVERTER_H_
#define INCLUDE_HQHP_BITCONVERTER_H_

/*!
****************************************************************************************************
* 包含文件
****************************************************************************************************
*/
#include <hqhp/defs.h>
#ifdef __cplusplus
extern "C" {
#endif

/*!
****************************************************************************************************
* 常量定义
****************************************************************************************************
*/

/*!
****************************************************************************************************
* 类型定义
****************************************************************************************************
*/

/*!
****************************************************************************************************
* 全局变量
****************************************************************************************************
*/

/*!
****************************************************************************************************
* 接口函数
****************************************************************************************************
*/
void BitConverter_Int8ToBytes(int8_t value, uint8_t* buffer, size_t* index);
void BitConverter_Int16ToBytes(int16_t value, uint8_t* buffer, size_t* index);
void BitConverter_Int32ToBytes(int32_t value, uint8_t* buffer, size_t* index);
void BitConverter_UInt8ToBytes(uint8_t value, uint8_t* buffer, size_t* index);
void BitConverter_UInt16ToBytes(uint16_t value, uint8_t* buffer, size_t* index);
void BitConverter_UInt32ToBytes(uint32_t value, uint8_t* buffer, size_t* index);
void BitConverter_SingleToBytes(float value, uint8_t* buffer, size_t* index);
void BitConverter_DoubleToBytes(double value, uint8_t* buffer, size_t* index);

void BitConverter_BytesToInt16(int16_t* value, const uint8_t* buffer, size_t* index);
void BitConverter_BytesToInt32(int32_t* value, const uint8_t* buffer, size_t* index);
void BitConverter_BytesToUInt16(uint16_t* value, const uint8_t* buffer, size_t* index);
void BitConverter_BytesToUInt32(uint32_t* value, const uint8_t* buffer, size_t* index);
void BitConverter_BytesToSingle(float* value, const uint8_t* buffer, size_t* index);
void BitConverter_BytesToDouble(double* value, const uint8_t* buffer, size_t* index);

/*!
****************************************************************************************************
* 内联函数
****************************************************************************************************
*/
#ifdef __cplusplus
}
#endif
#endif /* INCLUDE_HQHP_BITCONVERTER_H_ */
