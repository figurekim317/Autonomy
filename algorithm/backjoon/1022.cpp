#include<iostream>
#include<cstdio>
#include<stdlib.h>

using namespace std;

int dx[4] = {-1, 0, 1, 0};
int dy[4] = {0, -1, 0, 1};

int main(){
	int r1, c1, r2, c2;
	int x, y, n, dir, next, cnt, dcnt, num;
	int map[50][5];
	cin>>r1>>c1>>r2>>c2;

	for(int i=0; i<=r2-r1; i++)
  {
		for(int j=0; j<=c2-c1; j++)
			map[i][j] = 0;
	}

	x = y= 0;
	dir = 3;
	dcnt = num = 1;
	cnt = 0;

  while(!((map[0][0] != 0 ) && (map[0][c2-c1] != 0 ) && (map[r2-r1][0] != 0 ) && (map[r2-r1][c2-c1] != 0)))
  {
		if(x-r1 >=0 && x-r1 <= (r2-r1) && y-c1 >=0 && y-c1 <= (c2-c1))
    {
			map[x-r1][y-c1] =num;
		}
		num++;
		cnt++;
		x = x+dx[dir];
		y = y+dy[dir];

    if(cnt == dcnt)
    {
			cnt=0;
			dir = (dir+1)%4;
			if(dir == 3 || dir == 1)
        dcnt++;
		}
	}
	cnt=0;
	while(num > 0)
  { 
		num /= 10;
		cnt++;
	}

	for(int i=0; i<=(r2-r1); i++)
  {
		for(int j=0; j<=(c2-c1); j++)
			printf("%*d ",cnt, map[i][j]);
		printf("\n");
	}
	return 0;
}
