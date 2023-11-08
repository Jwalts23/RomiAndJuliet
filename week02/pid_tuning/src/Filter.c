#include"Filter.h"


double rollingAverage(float DistancePulse)
{
  sum=0.0;
  average=0.0;

 if(k < 5)
 { 
   d[k]=DistancePulse;
   k++;   
 }
 else{
   
   sum= d[0]+d[1]+d[2]+d[3]+d[4];
   average= sum/5.0;
   d[0]=d[1];
   d[1]=d[2];
   d[2]=d[3];
   d[3]=d[4];
   d[4]=DistancePulse;
 }
 return average;
}
 
 void sortArray(double arr[])
 {

double temp;
  for(int i=0;i< 5;i++)
  {
    for(int j=i+1;j<4;j++)
    {
      if(arr[i] >arr[j] )
      {
        temp=arr[i];
        arr[i]=arr[j];
        arr[j]=temp;
      }
    }
  }
 }

 double rollingMedian(float distance)
 {
   median=0.0;
   if(i < 5)
 { 
   c[i]=distance;
   e[i]=distance;
   i++;
       
 }
 else if(i==5)
 {
   sortArray(e);
   median=e[2];
   i++;
 }

 else{
   // pushing the new value into the array
   c[0]=c[1];
   c[1]=c[2];
   c[2]=c[3];
   c[3]=c[4];
   c[4]=distance;

  e[0]=c[0];
  e[1]=c[1];
  e[2]=c[2];
  e[3]=c[3];
  e[4]=c[4];
   sortArray(e);
   median=e[2];
   
 }

return median;

 }

