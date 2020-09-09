#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <limits>
#include <vector>
#include <utility>
#include <math.h>

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

using namespace std;

int size;
char** board;
double** u;
double** pu;
double gam = 0.9;
double eps = 0.01;
pair<int, int> directions[] = {
    make_pair(-1,0), make_pair(1,0),
    make_pair(0,1), make_pair(0,-1),
};


bool isDestination(pair<int, int> s) {
    return board[s.first][s.second] == '.';
}


bool isObstacle(pair<int, int> s) {
    return board[s.first][s.second] == 'o';
}


double getReward(pair<int, int> s) {
    double reward = -1.0;
    if(board[s.first][s.second] == 'o') {
        reward -= 100.0;
    }
    else if(board[s.first][s.second] == '.') {
        reward += 100.0;
    }
    return reward;
}


char a2c(pair<int, int> a) {
    char ch;
    if(a == make_pair(-1,0)) {
        ch = '^';
    }
    else if(a == make_pair(1,0)) {
        ch = 'v';
    }
    else if(a == make_pair(0,1)) {
        ch = '>';
    }
    else {
        ch = '<';
    }
    return ch;
}


double getUtility(pair<int, int> s, pair<int, int> a) {
    double utility = 0.0;
    for(pair<int, int> direction : directions) {
        double prob = (direction == a) ? 0.7 : 0.1;
        int endx = s.first + direction.first;
        int endy = s.second + direction.second;
        
        if(endx < 0) endx = 0;
        else if(endx >= size) endx = size - 1;
        if(endy < 0) endy = 0;
        else if(endy >= size) endy = size - 1;
        
        utility += prob * u[endx][endy];
    }
    return utility;
}


void valueIteration() {
    while(true) {
        double delta = 0.0;
        for(int i = 0; i < size; i++) {
            for(int j = 0; j < size; j++) {
                pair<int, int> s = make_pair(i, j);
                double maxUtility = -INFINITY;
                for(pair<int, int> a : directions) {
                    maxUtility = MAX(maxUtility, getUtility(s, a));
                }
                if(isDestination(s)) {
                    pu[i][j] = getReward(s);
                }
                else {
                    pu[i][j] = getReward(s) + gam * maxUtility;
                }
                delta = MAX(delta, abs(pu[i][j] - u[i][j]));
            }
        }
        for(int i = 0; i < size; i++) {
            for(int j = 0; j < size; j++) {
                u[i][j] = pu[i][j];
            }
        }
        if(delta < eps) {
            break;
        }
    }
}


void computeActions() {
    for(int i = 0; i < size; i++) {
        for(int j = 0; j < size; j++) {
            pair<int, int> s = make_pair(i, j);
            if(isObstacle(s) || isDestination(s)) {
                continue;
            }
            
            pair<int, int> bestAction;
            double bestUtility = -INFINITY;
            for(pair<int, int> a : directions) {
                double dirUtility = getUtility(s, a);
                if(dirUtility > bestUtility + 1e-14) {
                    bestAction = a;
                    bestUtility = dirUtility;
                }
            }
            
            board[i][j] = a2c(bestAction);
        }
    }
}


void allocate() {
    board = new char*[size];
    u = new double*[size];
    pu = new double*[size];
    for(int i = 0; i < size; i++) {
        board[i] = new char[size];
        u[i] = new double[size];
        pu[i] = new double[size];
        for(int j = 0; j < size; j++) {
            board[i][j] = ' ';
            u[i][j] = 0;
            pu[i][j] = 0;
        }
    }
}


void deallocate() {
    for(int i = 0; i < size; i++) {
        delete [] board[i];
        delete [] u[i];
        delete [] pu[i];
    }
    delete [] board;
    delete [] u;
    delete [] pu;
}


int main() {
    ifstream fin("input.txt");
    
    // read initial numbers
    int numObstacles;
    fin >> size >> numObstacles;
    string line;
    getline(fin, line);
    
    // allocate arrays
    allocate();

    // read
    for(int i = 0; i <= numObstacles; i++) {
        getline(fin, line);
        stringstream ss(line);
        int x, y;
        char ch;
        ss >> x >> ch >> y;
        if(i == numObstacles) {
            board[y][x] = '.';
        }
        else {
            board[y][x] = 'o';
        }
    }

    fin.close();

    valueIteration();
    
    computeActions();
    
    // write output to file
    ofstream fout("output.txt");
    for(int i = 0; i < size; i++) {
        for(int j = 0; j < size; j++) {
            fout << board[i][j];
        }
        fout << endl;
    }
    fout.close();
    
    // deallocate arrays
    deallocate();

    return 0;
}
