#include <iostream>
#include <string>
#include <utility>
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <sstream>
#include <iomanip>

using namespace std;

using State = vector< pair<int, int> >;
using Action = pair<int, int>;
using Drone = pair<int, int>;
using Coord = pair<int, int>;

// #define DEBUG

struct CoordHash {
    size_t operator()(const Coord &coord) const {
        return hash<int>()(coord.first) ^ hash<int>()(coord.second);
    }
};

struct StateHash {
    size_t operator()(const State &state) const {
        size_t x = state.size();
        for(Coord coord : state) {
            x ^= CoordHash()(coord) + 0x9e3779b9 + (x << 6) + (x >> 2);
        }
        return x;
    }
};

class Problem {
public:
    virtual vector<Action> actions(State state) = 0;
    virtual State step(State state, Action action) = 0;
    virtual int solve() = 0;
    virtual int getNumPackagesCollected(State state) = 0;
    virtual int getNumEligiblePackages(State state) = 0;
    virtual string getStateString(State state) = 0;
    virtual const int getN() = 0;
    virtual const int getD() = 0;
    virtual const int getXY(Coord coord) = 0;
};

class Solver {
public:
    Solver(Problem *problem) : problem(problem) {}

    virtual int solve() = 0;

protected:
    Problem* problem;
};

class DfsSolver : public Solver {
public:
    DfsSolver(Problem* problem) : Solver(problem) {}

    int solve() {
        State initState;
        return helper(initState, 0);
    }

private:
    int helper(State state, int optSoFar) {
        #ifdef DEBUG
        cout << problem->getStateString(state) << endl;
        #endif

        if(state.size() == problem->getD()) {
            return problem->getNumPackagesCollected(state);
        }

        int currentUpperBound = problem->getNumPackagesCollected(state) 
            + problem->getNumEligiblePackages(state);

        // check if we will never reach optimal state from here
        if(currentUpperBound <= optSoFar) {
            return 0;
        }

        vector<Action> eligibleActions = problem->actions(state);

        sort(eligibleActions.begin(), eligibleActions.end(), 
            [this](Action a1, Action a2) {
                return problem->getXY(a1) > problem->getXY(a2);
            });

        for(Action action : eligibleActions) {
            State nextState = problem->step(state, action);
            optSoFar = max(optSoFar, helper(nextState, optSoFar));
            if(optSoFar == currentUpperBound) break;
        }

        return optSoFar;
    }
};

class FasterDFSSolver : public Solver {
public:
    FasterDFSSolver(Problem* problem) : Solver(problem), numPlaced(0), 
                numPackagesCollected(0), numExpansions(0) {
        n = problem -> getN();
        d = problem -> getD();
        packages = vector< vector<int> >(n, vector<int>(n, 0));
        covered = vector< vector<int> >(n, vector<int>(n, -1));
        placed = vector< vector<bool> >(n, vector<bool>(n, false));
        for(int i = 0; i < n; i++) {
            for(int j = 0; j < n; j++) {
                packages[i][j] = problem -> getXY(make_pair(i,j));
            }
        }
    }
    
    int solve() {
        #ifdef DEBUG
        cout << problem->getStateString(history) << endl;
        #endif

        numExpansions++;

        if(numPlaced == d) {
            return numPackagesCollected;
        }

        int opt = 0;

        // int currentUpperBound = numPackagesCollected + getOverestimation();

        // if(currentUpperBound <= opt) {
        //     return 0;
        // }

        int startingRow = (history.empty()) ? 0 : (history[history.size() - 1].first + 1);
        int endRow = n - (d - numPlaced);
        for(int i = startingRow; i <= endRow; i++) {
            for(int j = 0; j < n; j++) {
                if(covered[i][j] == -1) {
                    // stepping
                    step(make_pair(i,j));
                    opt = max(opt, solve());
                    unstep(make_pair(i,j));
                }
            }
        }

        return opt;
    }

    int getNumExpansions() {
        return numExpansions;
    }

private:
    vector< vector<int> > packages;
    vector< vector<int> > covered;
    vector< vector<bool> > placed;
    vector< Action > history;
    int n;
    int d;
    int numPlaced;
    int numPackagesCollected;
    int numExpansions;

    void step(Action action) {
        // update cover matrix
        for(int j = 0; j < n; j++) {
            if(covered[action.first][j] == -1)
                covered[action.first][j] = numPlaced;
        }
        for(int i = 0; i < n; i++) {
            if(covered[i][action.second] == -1)
                covered[i][action.second] = numPlaced;
        }
        for(int i = 0; i < n; i++) {
            int j = action.first + action.second - i;
            if(j >= 0 && j < n) {
                if(covered[i][j] == -1)
                    covered[i][j] = numPlaced;
            }
            j = i - action.first + action.second;
            if(j >= 0 && j < n) {
                if(covered[i][j] == -1)
                    covered[i][j] = numPlaced;
            }
        }
        
        // update drone matrix
        placed[action.first][action.second] = true;

        // update history
        history.push_back(action);

        // update numPlaced
        numPlaced++;

        // update numPackagesCollected
        numPackagesCollected += packages[action.first][action.second];
    }

    void unstep(Action action) {
        for(int i = 0; i < n; i++) {
            for(int j = 0; j < n; j++) {
                if(covered[i][j] == numPlaced - 1) {
                    covered[i][j] = -1;
                }
            }
        }

        placed[action.first][action.second] = false;

        history.pop_back();

        numPlaced--;

        numPackagesCollected -= packages[action.first][action.second];
    }

    int getOverestimation() {
        priority_queue<int> pq;
        for(int i = 0; i < n; i++) {
            int maxPackageCount = 0;
            bool dronePlaced = false;
            for(int j = 0; j < n; j++) {
                int coordPackageCount = packages[i][j];
                if(covered[i][j] == -1
                    && (coordPackageCount > maxPackageCount)) {
                    maxPackageCount = coordPackageCount;
                }
                if(placed[i][j]) {
                    dronePlaced = true;
                    break;
                }
            }
            if(!dronePlaced) {
                pq.push(maxPackageCount);
            }
        }

        int res = 0;
        int numDronesLeft = d - history.size();
        for(int i = 0; i < numDronesLeft; i++) {
            res += pq.top();
            pq.pop();
        }
        return res;
    }
};

class AstarSolver : public Solver {
public:
    AstarSolver(Problem* problem) : Solver(problem) {}

    int solve() {
        State initState;
        auto comp = [this](State s1, State s2) {
            int g1 = problem->getNumPackagesCollected(s1);
            int g2 = problem->getNumPackagesCollected(s2);
            int f1 = g1 + heuristic(s1);
            int f2 = g2 + heuristic(s2);
            if(f1 != f2) {
                return f1 < f2;
            }
            return g1 < g2;
        };
        priority_queue<State, vector<State>, decltype(comp)> frontier(comp);
        frontier.push(initState);
        unordered_set<State, StateHash> frontierSet;
        unordered_set<State, StateHash> explored;
        while(true) {
            if(frontier.empty()) {
                // failure
                return 0;
            }
            State state = frontier.top();
            frontier.pop();
            frontierSet.erase(state);

            #ifdef DEBUG
            cout << problem->getStateString(state) << endl;
            #endif

            if(problem->getD() == state.size()) {
                return problem->getNumPackagesCollected(state);
            }
            explored.insert(state);
            for(Action action : problem->actions(state)) {
                State nextState = problem->step(state, action);

                if(explored.find(nextState) == explored.end()
                    && frontierSet.find(nextState) == frontierSet.end()) {
                    frontier.push(nextState);
                    frontierSet.insert(nextState);
                }
            }
        }
    }

private:
    int heuristic(State state) {
        return problem->getNumEligiblePackages(state);
    }
};

class DroneProblem : public Problem {
public:
    DroneProblem(int n, int d, vector<Coord> coords, string algorithm)
            : n(n), d(d), algorithm(algorithm) {
        for(Coord coord : coords) {
            auto it = packages.find(coord);
            if(it == packages.end()) {
                packages.insert(make_pair(coord, 1));
            }
            else {
                it->second++;
            }
        }
    }

    vector<Action> actions(State state) {
        vector<Action> eligibleActions;
        int startingRow = (state.empty()) ? 0 : (state[state.size() - 1].first + 1);
        int endRow = n - (d - state.size());
        for(int i = startingRow; i <= endRow; i++) {
            for(int j = 0; j < n; j++) {
                Action action = make_pair(i, j);
                if(!checkCovered(state, action)) {
                    eligibleActions.push_back(action);
                }
            }
        }
        return eligibleActions;
    }

    State step(State state, Action action) {
        State nextState(state);
        nextState.push_back(action);
        return nextState;
    }

    int solve() {
        if(algorithm == "dfs") {
            return FasterDFSSolver(this).solve();
        }
        else if(algorithm == "astar") {
            return AstarSolver(this).solve();
        }
        
        throw invalid_argument("algorithm must be dfs or astar.");
    }

    int getNumPackagesCollected(State state) {
        int numCollected = 0;
        for(Drone drone : state) {
            numCollected += getXY(drone);
        }
        return numCollected;
    }

    int getNumEligiblePackages(State state) {
        int eRow = getNumEligiblePackages(state, "row");
        int eCol = getNumEligiblePackages(state, "col");
        int eDiag1 = getNumEligiblePackages(state, "diag1");
        int eDiag2 = getNumEligiblePackages(state, "diag2");
        return min(min(eRow, eCol), min(eDiag1, eDiag2));
    }

    string getStateString(State state) {
        stringstream ss;
        ss << "Placed: " << setw(3) << state.size() << endl;
        ss << "Left: " << setw(3) << d - state.size() << endl;
        ss << "Score: " << setw(3) << getNumPackagesCollected(state) << endl;
        ss << "Eligb: " << setw(3) << getNumEligiblePackages(state) << endl;
        for(int i = 0; i < n; i++) {
            ss << separatorString() << endl;
            ss << rowString(state, i) << endl;
        }
        ss << separatorString() << endl;
        return ss.str();
    }

    const int getN() {
        return n;
    }

    const int getD() {
        return d;
    }

    const int getXY(Coord coord) {
        auto it = packages.find(coord);
        return (it != packages.end()) ? it->second : 0;
    }

private:
    int n; // board size
    int d; // number of drones
    unordered_map<Coord, int, CoordHash> packages; // list of coordinates
    string algorithm;

    /* Check whether a position in the board is covered by a drone. */
    bool checkCovered(State state, Coord coord) {
        for(Drone drone : state) {
            // same row
            if(drone.first == coord.first) return true;

            // same column
            if(drone.second == coord.second) return true;

            // same first diagonal
            if(drone.first - drone.second == coord.first - coord.second) {
                return true;
            }

            // same second diagonal
            if(drone.first + drone.second == coord.first + coord.second) {
                return true;
            }
        }

        return false;
    }

    int getNumEligiblePackages(State state, string variation) {
        priority_queue<int> pq;

        if(variation == "row") {
            for(int i = 0; i < n; i++) {
                int maxPackageCount = 0;
                bool dronePlaced = false;
                for(int j = 0; j < n; j++) {
                    Coord coord = make_pair(i, j);
                    int coordPackageCount = getXY(coord);
                    if((!checkCovered(state, coord)) 
                        && (coordPackageCount > maxPackageCount)) {
                        maxPackageCount = coordPackageCount;
                    }
                    if(find(state.begin(), state.end(), coord) != state.end()) {
                        dronePlaced = true;
                        break;
                    }
                }
                if(!dronePlaced) {
                    pq.push(maxPackageCount);
                }
            }
        }
        else if(variation == "col") {
            for(int j = 0; j < n; j++) {
                int maxPackageCount = 0;
                bool dronePlaced = false;
                for(int i = 0; i < n; i++) {
                    Coord coord = make_pair(i, j);
                    int coordPackageCount = getXY(coord);
                    if((!checkCovered(state, coord)) 
                        && (coordPackageCount > maxPackageCount)) {
                        maxPackageCount = coordPackageCount;
                    }
                    if(find(state.begin(), state.end(), coord) != state.end()) {
                        dronePlaced = true;
                        break;
                    }
                }
                if(!dronePlaced) {
                    pq.push(maxPackageCount);
                }
            }
        }
        else if(variation == "diag1") {
            for(int dist = -(n - 1); dist <= (n - 1); dist++) {
                int maxPackageCount = 0;
                bool dronePlaced = false;
                int iMin = max(0, dist);
                int iMax = min(n - 1, dist + n - 1);
                for(int i = iMin; i <= iMax; i++) {
                    int j = i - dist;
                    Coord coord = make_pair(i, j);
                    int coordPackageCount = getXY(coord);
                    if((!checkCovered(state, coord)) 
                        && (coordPackageCount > maxPackageCount)) {
                        maxPackageCount = coordPackageCount;
                    }
                    if(find(state.begin(), state.end(), coord) != state.end()) {
                        dronePlaced = true;
                        break;
                    }
                }
                if(!dronePlaced) {
                    pq.push(maxPackageCount);
                }
            }
        }
        else if(variation == "diag2") {
            for(int sum = 0; sum <= 2 * (n - 1); sum++) {
                int maxPackageCount = 0;
                bool dronePlaced = false;
                int iMin = max(0, sum - (n - 1));
                int iMax = min(n - 1, sum);
                for(int i = iMin; i <= iMax; i++) {
                    int j = sum - i;
                    Coord coord = make_pair(i, j);
                    int coordPackageCount = getXY(coord);
                    if((!checkCovered(state, coord)) 
                        && (coordPackageCount > maxPackageCount)) {
                        maxPackageCount = coordPackageCount;
                    }
                    if(find(state.begin(), state.end(), coord) != state.end()) {
                        dronePlaced = true;
                        break;
                    }
                }
                if(!dronePlaced) {
                    pq.push(maxPackageCount);
                }
            }
        }
        else {
            throw invalid_argument("algorithm must be in [row, col, diag1, diag2].");
        }

        int res = 0;
        int numDronesLeft = d - state.size();
        for(int i = 0; i < numDronesLeft; i++) {
            res += pq.top();
            pq.pop();
        }
        return res;
    }

    string separatorString() {
        stringstream ss;
        for(int i = 0; i < n; i++) {
            ss << "+----";
        }
        ss << "+";
        return ss.str();
    }

    string rowString(State state, int i) {
        stringstream ss;
        for(int j = 0; j < n; j++) {
            Coord coord = make_pair(i, j);
            int numPackages = getXY(coord);
            bool drone = find(state.begin(), state.end(), coord) != state.end();
            string droneMark = drone ? "x" : " ";
            ss << "|" << droneMark << " " 
               << setw(2);
            if(numPackages > 0) {
                ss << numPackages;
            }
            else {
                ss << "";
            }
        }
        ss << "|";
        return ss.str();
    }
};

int main(int argc, char *argv[]) {
    int n, d, p;
    cin >> n >> d >> p;

    // read past endline
    string line;
    getline(cin, line);

    string algorithm;
    getline(cin, algorithm);

    vector<Coord> coords;
    for(int i = 0; i < p; i++) {
        getline(cin, line);
        int delimPosition = line.find(',');
        int x = stoi(line.substr(0, delimPosition));
        int y = stoi(line.substr(delimPosition + 1));
        coords.push_back(make_pair(x, y));
    }

    DroneProblem problem(n, d, coords, algorithm);

    cout << problem.solve() << endl;

    return 0;
}
