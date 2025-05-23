/*!
****************************************************************************************************
* 文件名称：bitreverse.h
* 功能简介：该文件是位反转算法的接口头文件
* 文件作者：HQHP
* 创建日期：2023-01-31
* 版权声明：All Rights Reserved.
****************************************************************************************************
*/
#ifndef INCLUDE_HQHP_BITREVERSE_H_
#define INCLUDE_HQHP_BITREVERSE_H_

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

/*!
****************************************************************************************************
* 内联函数
****************************************************************************************************
*/
/*!
****************************************************************************************************
* 功能描述：该方法用于将32位无符号数进行位反转，即0x00000001将变为0x80000000
* 注意事项：算法来自Bit Twiddling Hacks
* 输入参数：x -- 待反转的32位无符号数
* 输出参数：NA
* 返回参数：反转后的32位无符号数
****************************************************************************************************
*/
static inline uint32_t BitReverse32(uint32_t x)
{
	x = (((x & 0xaaaaaaaa) >> 1) | ((x & 0x55555555) << 1));
	x = (((x & 0xcccccccc) >> 2) | ((x & 0x33333333) << 2));
	x = (((x & 0xf0f0f0f0) >> 4) | ((x & 0x0f0f0f0f) << 4));
	x = (((x & 0xff00ff00) >> 8) | ((x & 0x00ff00ff) << 8));

	return ((x >> 16) | (x << 16));
}

/*!
****************************************************************************************************
* 功能描述：该方法用于将16位无符号数进行位反转，即0x0001将变为0x8000
* 注意事项：算法来自Bit Twiddling Hacks
* 输入参数：x -- 待反转的16位无符号数
* 输出参数：NA
* 返回参数：反转后的16位无符号数
****************************************************************************************************
*/
static inline uint16_t BitReverse16(uint16_t x)
{
	uint32_t v = BitReverse32(x);

	return v >> 16;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于将8位无符号数进行位反转，即0x01将变为0x80
* 注意事项：算法来自Bit Twiddling Hacks
* 输入参数：x -- 待反转的8位无符号数
* 输出参数：NA
* 返回参数：反转后的8位无符号数
****************************************************************************************************
*/
static inline uint8_t BitReverse8(uint8_t x)
{
	uint16_t v = BitReverse16(x);

	return v >> 8;
}

#ifdef __cplusplus
}
#endif
#endif /* INCLUDE_HQHP_BITREVERSE_H_ */
