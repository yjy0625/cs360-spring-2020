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

// global variables
int numPlayers = 0;
string algorithm = "";

struct Player {
    int id;
    double c;
    double h1;
    double h2;
    int team;
};

// each player is represented by (id, c, h1, h2, team)
vector<Player> players;
int aCount = 0;
int bCount = 0;

double heuristic() {
    int aIds[10] = {0};
    int bIds[10] = {0};
    double score = 0.0;
    bool ad = true; // a team is diverse
    bool bd = true; // b team is diverse
    for(Player player : players) {
        if(player.team == 1) {
            score += player.h1 * player.c;
            aIds[player.id % 10]++;
            if(aIds[player.id % 10] > 1) {
                ad = false;
            }
        }
        else if(player.team == 2) {
            score -= player.h2 * player.c;
            bIds[player.id % 10]++;
            if(bIds[player.id % 10] > 1) {
                bd = false;
            }
        }
    }
    if(ad) score += 120.0;
    if(bd) score -= 120.0;
    return score;
}

pair<int, double> alphabeta(double alpha, double beta, bool maximizing) {
    if(aCount == 5 && bCount == 5) {
        return make_pair(0, heuristic());
    }

    double score;
    int bestId;

    if(maximizing) {
        score = -INFINITY;
        for(int i = 0; i < numPlayers; i++) {
            Player player = players[i];
            if(player.team != 0) {
                continue;
            }
            players[i].team = 1;
            aCount++;
            double currScore = alphabeta(alpha, beta, false).second;
            if(currScore > score || (currScore == score && player.id < bestId)) {
                bestId = player.id;
                score = currScore;
            }
            alpha = MAX(alpha, score);
            aCount--;
            players[i].team = 0;
            if(alpha >= beta && algorithm == "ab") {
                break;
            }
        }
    }
    else {
        score = INFINITY;
        for(int i = 0; i < numPlayers; i++) {
            Player player = players[i];
            if(player.team != 0) {
                continue;
            }
            players[i].team = 2;
            bCount++;
            double currScore = alphabeta(alpha, beta, true).second;
            if(currScore < score || (currScore == score && player.id < bestId)) {
                bestId = player.id;
                score = currScore;
            }
            beta = MIN(beta, score);
            bCount--;
            players[i].team = 0;
            if(alpha >= beta && algorithm == "ab") {
                break;
            }
        }
    }

    return make_pair(bestId, score);
}

int main() {
    ifstream fin("input.txt");
    fin >> numPlayers;
    string line;
    getline(fin, line);
    getline(fin, algorithm);

    for(int i = 0; i < numPlayers; i++) {
        getline(fin, line);
        stringstream ss(line);
        int id; double c; double h1; double h2; int team;
        char ch;
        ss >> id >> ch >> c >> ch >> h1 >> ch >> h2 >> ch >> team;
        players.push_back({id, c, h1, h2, team});
        if(team == 1) {
            aCount++;
        }
        else if(team == 2) {
            bCount++;
        }
    }

    fin.close();

    int result = alphabeta(-INFINITY, INFINITY, true).first;
    ofstream fout("output.txt");
    fout << result << endl;
    fout.close();

    return 0;
}