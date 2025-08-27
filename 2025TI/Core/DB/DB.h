#ifndef __FILTER_DETECT_H
#define __FILTER_DETECT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FILTER_DATA_POINTS 495  // ��������


/**
 * @brief ����Ƶ����Ӧ˥�������ж��˲�������
 * @param att ˥�����飨��λ dB������ FILTER_DATA_POINTS ����
 * @return FilterType ö�٣���ʾ�˲�������
 */
char detect_filter_type(float att[495]);

/**
 * @brief ���˲�������ö��ת��Ϊ�ַ���
 * @param type ö������
 * @return const char* ��Ӧ�ַ���
 */

#ifdef __cplusplus
}
#endif

#endif // __FILTER_DETECT_H