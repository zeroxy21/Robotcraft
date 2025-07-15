//
// Created by karim on 03/07/2025.
//
#include <iostream>
#include "Position.h"
#include "math.h"
Position::Position(float x,float y) : x(x),y(y) {}

char Position::PrintPosition(Position* p){
    std::cout<<"Robot Position : x="<<p->x<<" y="<<p->y;
}
void Position::CreatePosition() {
    this->x = 0;
    this->y = 0;
}
void Position::CreatePosition(int x,int y){
    this->x = x;
    this->y = y;
}
void Position::DestroyPosition(Position p){
    delete p;
}
float Position::DistancePosition(Position p,Position q) {
    float dx = q.x - p.x;
    float dy = q.y - p.y;
    return sqrt(dx*dx + dy*dy);
}
int Position::GenerateArrayOfPositions(int N) {
        Position* arr = new Position[N];
        for (int i = 0; i < N; i++) {
            int rand_x = rand() % 10;
            int rand_y = rand() % 10;
            arr[i] = Position(rand_x, rand_y);
        }

        return arr;

    }
void Position::DestroyArrayOfPositions(int array[]) {
    delete[] array;
}




