//
// Created by karim on 03/07/2025.
//

#ifndef POSITION_H
#define POSITION_H



class Position {

private:
    float x,y;
public:
    Position(float x,float y);
    char PrintPosition(Position* p);
    void CreatePosition();
    void CreatePosition(int x,int y);
    void DestroyPosition(Position p);
    float DistancePosition(Position p,Position q);
    int GenerateArrayOfPositions(int N);
    void DestroyArrayOfPositions(int array[]);
};



#endif //POSITION_H
