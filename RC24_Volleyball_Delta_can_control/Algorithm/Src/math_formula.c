#include "math_formula.h"

/**************************************************************
	@brief:		�޷�����
	@param:		
	@retval: 		
	@supplement:	
**************************************************************/
void limit_amplitude(float *a, float *limit)
{
	if(*a > *limit)
		*a = *limit;
	else if(*a < -*limit)
		*a = -*limit;
}

/**************************************************************
	@brief:		ţ�ٷ���ƽ����ֵ
	@param:		
	@retval: 		
	@supplement:	
**************************************************************/
float InvSqrt(float x) 
{
 	float xhalf = 0.5f*x; 	int i = *(int*)&x; // get bits for floating VALUE 	
	i = 0x5f375a86- (i>>1); // gives initial guess y0 	
	x = *(float*)&i; // convert bits BACK to float 	
	x = x*(1.5f-xhalf*x*x); // Newton step, repeating increases accuracy 	
	x = x*(1.5f-xhalf*x*x); // Newton step, repeating increases accuracy 	
	x = x*(1.5f-xhalf*x*x); // Newton step, repeating increases accuracy 	
	return 1/x; 
}

/**************************************************************
	@brief:		ð������
	@param:		
	@retval: 		
	@supplement:	
**************************************************************/
int Bubble_Sort(int data[], int n)
{
	for(int i=0;i<n-1;i++)//��������ѭ������n�����ͱȽ�n-1��
 {
	 for(int j=0;j<n-1-i;j++)//������ѭ����ÿ�ֱȽ�n-1-����һ�ֵĳ�ȥ�����ĸ�����
	 {
		 if(data[j]>data[j+1])//�ж������ߵ��������ұߵ������ͰѴ����������
     {
			 //����ǽ������õ����������ķ�ʽ����ȻҲ���Խ��õ���������������
       data[j]=data[j]+data[j+1];
       data[j+1]=data[j]-data[j+1];
       data[j]=data[j]-data[j+1];
		 }
	 }
 }
 return data[0];
}


/**************************************************************
	@brief:		����ֵ����
	@param:		
	@retval: 		
	@supplement:	
**************************************************************/
float absoluteValue(float x)
{
    if (x < 0)	return -x;
    else 	return x;
}


