/*!
****************************************************************************************************
* 文件名称：bitconverter.c
* 功能简介：该文件是字节流转换模块的接口头文件
* 文件作者：Haotian
* 创建日期：2020-09-25
* 版权声明：All Rights Reserved.
****************************************************************************************************
*/
#include <hqhp/endian.h>
#include <hqhp/bitconverter.h>

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
* 本地声明
****************************************************************************************************
*/

/*!
****************************************************************************************************
* 接口函数
****************************************************************************************************
*/
/*!
****************************************************************************************************
* 功能描述：
* 注意事项：
* 输入参数：
* 输出参数：
* 返回参数：
****************************************************************************************************
*/
void BitConverter_Int8ToBytes(int8_t value, uint8_t* buffer, size_t* index)
{
	buffer[*index] = value;
	*index += 1;
}

/*!
****************************************************************************************************
* 功能描述：
* 注意事项：
* 输入参数：
* 输出参数：
* 返回参数：
****************************************************************************************************
*/
void BitConverter_Int16ToBytes(int16_t value, uint8_t* buffer, size_t* index)
{
	size_t tmpIndex = *index;
	union {
		uint8_t Buf[2];
		int16_t Value;
	} U;

	U.Value = value;
	if (Endian_isBigEndian() == true) {
		buffer[tmpIndex++] = U.Buf[0];
		buffer[tmpIndex++] = U.Buf[1];
	} else {
		buffer[tmpIndex++] = U.Buf[1];
		buffer[tmpIndex++] = U.Buf[0];
	}

	*index = tmpIndex;
}

/*!
****************************************************************************************************
* 功能描述：
* 注意事项：
* 输入参数：
* 输出参数：
* 返回参数：
****************************************************************************************************
*/
void BitConverter_Int32ToBytes(int32_t value, uint8_t* buffer, size_t* index)
{
	size_t tmpIndex = *index;
	union {
		uint8_t Buf[4];
		int32_t Value;
	} U;

	U.Value = value;
	if (Endian_isBigEndian() == true) {
		buffer[tmpIndex++] = U.Buf[0];
		buffer[tmpIndex++] = U.Buf[1];
		buffer[tmpIndex++] = U.Buf[2];
		buffer[tmpIndex++] = U.Buf[3];
	} else {
		buffer[tmpIndex++] = U.Buf[3];
		buffer[tmpIndex++] = U.Buf[2];
		buffer[tmpIndex++] = U.Buf[1];
		buffer[tmpIndex++] = U.Buf[0];
	}

	*index = tmpIndex;
}

/*!
****************************************************************************************************
* 功能描述：
* 注意事项：
* 输入参数：
* 输出参数：
* 返回参数：
****************************************************************************************************
*/
void BitConverter_UInt8ToBytes(uint8_t value, uint8_t* buffer, size_t* index)
{
	buffer[*index] = value;
	*index += 1;
}

/*!
****************************************************************************************************
* 功能描述：
* 注意事项：
* 输入参数：
* 输出参数：
* 返回参数：
****************************************************************************************************
*/
void BitConverter_UInt16ToBytes(uint16_t value, uint8_t* buffer, size_t* index)
{
	size_t tmpIndex = *index;
	union {
		uint8_t	 Buf[2];
		uint16_t Value;
	} U;

	U.Value = value;
	if (Endian_isBigEndian() == true) {
		buffer[tmpIndex++] = U.Buf[0];
		buffer[tmpIndex++] = U.Buf[1];
	} else {
		buffer[tmpIndex++] = U.Buf[1];
		buffer[tmpIndex++] = U.Buf[0];
	}

	*index = tmpIndex;
}

/*!
****************************************************************************************************
* 功能描述：
* 注意事项：
* 输入参数：
* 输出参数：
* 返回参数：
****************************************************************************************************
*/
void BitConverter_UInt32ToBytes(uint32_t value, uint8_t* buffer, size_t* index)
{
	size_t tmpIndex = *index;
	union {
		uint8_t	 Buf[4];
		uint32_t Value;
	} U;

	U.Value = value;
	if (Endian_isBigEndian() == true) {
		buffer[tmpIndex++] = U.Buf[0];
		buffer[tmpIndex++] = U.Buf[1];
		buffer[tmpIndex++] = U.Buf[2];
		buffer[tmpIndex++] = U.Buf[3];
	} else {
		buffer[tmpIndex++] = U.Buf[3];
		buffer[tmpIndex++] = U.Buf[2];
		buffer[tmpIndex++] = U.Buf[1];
		buffer[tmpIndex++] = U.Buf[0];
	}

	*index = tmpIndex;
}

/*!
****************************************************************************************************
* 功能描述：
* 注意事项：
* 输入参数：
* 输出参数：
* 返回参数：
****************************************************************************************************
*/
void BitConverter_SingleToBytes(float value, uint8_t* buffer, size_t* index)
{
	size_t tmpIndex = *index;
	union {
		uint8_t Buf[4];
		float	Value;
	} U;

	U.Value = value;
	if (Endian_isBigEndian() == true) {
		buffer[tmpIndex++] = U.Buf[0];
		buffer[tmpIndex++] = U.Buf[1];
		buffer[tmpIndex++] = U.Buf[2];
		buffer[tmpIndex++] = U.Buf[3];
	} else {
		buffer[tmpIndex++] = U.Buf[3];
		buffer[tmpIndex++] = U.Buf[2];
		buffer[tmpIndex++] = U.Buf[1];
		buffer[tmpIndex++] = U.Buf[0];
	}

	*index = tmpIndex;
}

/*!
****************************************************************************************************
* 功能描述：
* 注意事项：
* 输入参数：
* 输出参数：
* 返回参数：
****************************************************************************************************
*/
void BitConverter_DoubleToBytes(double value, uint8_t* buffer, size_t* index)
{
	size_t tmpIndex = *index;

	union {
		uint8_t Buf[8];
		double	Value;
	} U;

	U.Value = value;
	if (Endian_isBigEndian() == true) {
		buffer[tmpIndex++] = U.Buf[0];
		buffer[tmpIndex++] = U.Buf[1];
		buffer[tmpIndex++] = U.Buf[2];
		buffer[tmpIndex++] = U.Buf[3];
		buffer[tmpIndex++] = U.Buf[4];
		buffer[tmpIndex++] = U.Buf[5];
		buffer[tmpIndex++] = U.Buf[6];
		buffer[tmpIndex++] = U.Buf[7];
	} else {
		buffer[tmpIndex++] = U.Buf[7];
		buffer[tmpIndex++] = U.Buf[6];
		buffer[tmpIndex++] = U.Buf[5];
		buffer[tmpIndex++] = U.Buf[4];
		buffer[tmpIndex++] = U.Buf[3];
		buffer[tmpIndex++] = U.Buf[2];
		buffer[tmpIndex++] = U.Buf[1];
		buffer[tmpIndex++] = U.Buf[0];
	}

	*index = tmpIndex;
}

/*!
****************************************************************************************************
* 功能描述：
* 注意事项：
* 输入参数：
* 输出参数：
* 返回参数：
****************************************************************************************************
*/
void BitConverter_BytesToInt16(int16_t* value, const uint8_t* buffer, size_t* index)
{
	union {
		uint8_t Buf[2];
		int16_t Value;
	} U;

	if (Endian_isBigEndian() == true) {
		U.Buf[0] = buffer[index[0] + 0];
		U.Buf[1] = buffer[index[0] + 1];
	} else {
		U.Buf[0] = buffer[index[0] + 1];
		U.Buf[1] = buffer[index[0] + 0];
	}

	value[0] = U.Value;
	index[0] += 2;
}

/*!
****************************************************************************************************
* 功能描述：
* 注意事项：
* 输入参数：
* 输出参数：
* 返回参数：
****************************************************************************************************
*/
void BitConverter_BytesToInt32(int32_t* value, const uint8_t* buffer, size_t* index)
{
	union {
		uint8_t Buf[4];
		int32_t Value;
	} U;

	if (Endian_isBigEndian() == true) {
		U.Buf[0] = buffer[index[0] + 0];
		U.Buf[1] = buffer[index[0] + 1];
		U.Buf[2] = buffer[index[0] + 2];
		U.Buf[3] = buffer[index[0] + 3];
	} else {
		U.Buf[0] = buffer[index[0] + 3];
		U.Buf[1] = buffer[index[0] + 2];
		U.Buf[2] = buffer[index[0] + 1];
		U.Buf[3] = buffer[index[0] + 0];
	}

	value[0] = U.Value;
	index[0] += 4;
}

/*!
****************************************************************************************************
* 功能描述：
* 注意事项：
* 输入参数：
* 输出参数：
* 返回参数：
****************************************************************************************************
*/
void BitConverter_BytesToUInt16(uint16_t* value, const uint8_t* buffer, size_t* index)
{
	union {
		uint8_t	 Buf[2];
		uint16_t Value;
	} U;

	if (Endian_isBigEndian() == true) {
		U.Buf[0] = buffer[index[0] + 0];
		U.Buf[1] = buffer[index[0] + 1];
	} else {
		U.Buf[0] = buffer[index[0] + 1];
		U.Buf[1] = buffer[index[0] + 0];
	}

	value[0] = U.Value;
	index[0] += 2;
}

/*!
****************************************************************************************************
* 功能描述：
* 注意事项：
* 输入参数：
* 输出参数：
* 返回参数：
****************************************************************************************************
*/
void BitConverter_BytesToUInt32(uint32_t* value, const uint8_t* buffer, size_t* index)
{
	union {
		uint8_t	 Buf[4];
		uint32_t Value;
	} U;

	if (Endian_isBigEndian() == true) {
		U.Buf[0] = buffer[index[0] + 0];
		U.Buf[1] = buffer[index[0] + 1];
		U.Buf[2] = buffer[index[0] + 2];
		U.Buf[3] = buffer[index[0] + 3];
	} else {
		U.Buf[0] = buffer[index[0] + 3];
		U.Buf[1] = buffer[index[0] + 2];
		U.Buf[2] = buffer[index[0] + 1];
		U.Buf[3] = buffer[index[0] + 0];
	}

	value[0] = U.Value;
	index[0] += 4;
}

/*!
****************************************************************************************************
* 功能描述：
* 注意事项：
* 输入参数：
* 输出参数：
* 返回参数：
****************************************************************************************************
*/
void BitConverter_BytesToSingle(float* value, const uint8_t* buffer, size_t* index)
{
	union {
		uint8_t Buf[4];
		float	Value;
	} U;

	if (Endian_isBigEndian() == true) {
		U.Buf[0] = buffer[index[0] + 0];
		U.Buf[1] = buffer[index[0] + 1];
		U.Buf[2] = buffer[index[0] + 2];
		U.Buf[3] = buffer[index[0] + 3];
	} else {
		U.Buf[0] = buffer[index[0] + 3];
		U.Buf[1] = buffer[index[0] + 2];
		U.Buf[2] = buffer[index[0] + 1];
		U.Buf[3] = buffer[index[0] + 0];
	}

	value[0] = U.Value;
	index[0] += 4;
}

/*!
****************************************************************************************************
* 功能描述：
* 注意事项：
* 输入参数：
* 输出参数：
* 返回参数：
****************************************************************************************************
*/
void BitConverter_BytesToDouble(double* value, const uint8_t* buffer, size_t* index)
{
	union {
		uint8_t Buf[8];
		double	Value;
	} U;

	if (Endian_isBigEndian() == true) {
		U.Buf[0] = buffer[index[0] + 0];
		U.Buf[1] = buffer[index[0] + 1];
		U.Buf[2] = buffer[index[0] + 2];
		U.Buf[3] = buffer[index[0] + 3];
		U.Buf[4] = buffer[index[0] + 4];
		U.Buf[5] = buffer[index[0] + 5];
		U.Buf[6] = buffer[index[0] + 6];
		U.Buf[7] = buffer[index[0] + 7];
	}

	else {
		U.Buf[0] = buffer[index[0] + 7];
		U.Buf[1] = buffer[index[0] + 6];
		U.Buf[2] = buffer[index[0] + 5];
		U.Buf[3] = buffer[index[0] + 4];
		U.Buf[4] = buffer[index[0] + 3];
		U.Buf[5] = buffer[index[0] + 2];
		U.Buf[6] = buffer[index[0] + 1];
		U.Buf[7] = buffer[index[0] + 0];
	}

	value[0] = U.Value;
	index[0] += 8;
}

/*!
****************************************************************************************************
* 本地函数
****************************************************************************************************
*/
/*!
****************************************************************************************************
* 功能描述：
* 注意事项：
* 输入参数：
* 输出参数：
* 返回参数：
****************************************************************************************************
*/
