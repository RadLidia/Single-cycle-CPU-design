#include<stdio.h>
#include<stdlib.h>
#include<time.h>
#include<stdint.h>
#define N 100000

struct DATA {
  uint8_t a;
  int b;
  char c;
};

int main(void){
  struct DATA * pMyData= (struct DATA*)malloc(N*sizeof(struct DATA));
  long int i;
  double avg=0;

  if(pMyData ==NULL){
    printf("Insufficient memory.\n");
    return 0;
  }

  for(i=0; i<N; i++){    // Initialization of data fields of structures in the array
      pMyData[i].a = 0;
      pMyData[i].b = 0;
      pMyData[i].c = 0;
  }

  // HERE: Start measuring run time of the program

  // ... Some intermediate calculations could follow, using pMyData..


  for(i=0; i<N; i++)     // Increment the b variable.
      pMyData[i].b++;


  for(i=0; i<N; i+=2)     // and compute the average of every other value of variable "a" (even indices).
      avg+=pMyData[i].a;

  avg /= ((N/2)+(N%2));

  // HERE: End measuring run time of the program
  return 0;
}