#include<iostream>
using namespace std;


void tao_file(){
    freopen("data.inp", "w", stdout);
    srand(time(NULL));
    n = rand()%98+3;
    cout << n << endl;
}