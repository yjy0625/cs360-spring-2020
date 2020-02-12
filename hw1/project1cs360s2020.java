import java.io.*;
import java.util.*;
import java.util.stream.Collectors;

class Problem {
  private int boardSize;
  private State initState;

  public Problem(State state, String algorithm) {
    this.boardSize = state.getBoard().size();
    this.initState = state;
  }

  public State getInitState() {
    return initState;
  }

  public List<Action> actions(State state) {
    List<Action> eligibleActions = new ArrayList<>();
    int boardSize = state.getBoard().size();
    for(int i = 0; i < boardSize; i++) {
      for(int j = 0; j < boardSize; j++) {
        if(!state.getBoard().get(i).get(j).getCovered()) {
          eligibleActions.add(new Action(i, j));
        }
      }
    }
    return eligibleActions;
  }

  public State step(State state, Action action) {
    State nextState = new State(state);
    nextState.step(action);
    return nextState;
  }

  public int solve(String algorithm) {
    Solver solver;
    if(algorithm.equals("dfs")) {
      solver = new DfsSolver(this);
    }
    else if(algorithm.equals("astar")) {
      solver = new AstarSolver(this);
    }
    else {
      System.out.println("Invalid argument algorithm -- should be dfs or astar.");
      return 0;
    }
    return solver.solve();
  }
}

abstract class Solver {
  protected Problem problem;
  
  public Solver(Problem problem) {
    this.problem = problem;
  }

  protected void debug(State state) {
    System.out.println(state);
    try {
      Thread.sleep(1000);
    }
    catch(Exception e) {
      System.out.println("Input error.");
    }
  }

  abstract public int solve();
}

class DfsSolver extends Solver {
  public DfsSolver(Problem problem) {
    super(problem);
  }

  public int solve() {
    return helper(problem.getInitState(), 0);
  }

  private int helper(State state, int optSoFar) {
    // debug(state);

    if(state.getNumDronesLeft() == 0) {
      return state.getNumPackagesCovered();
    }

    // check if we will never reach optimal state from here
    if(state.getNumPackagesCovered() + state.getNumEligiblePackages() <= optSoFar) {
      return 0;
    }

    List<Action> eligibleActions = problem.actions(state);

    // sort actions so that we explore the ones covering most packages first
    eligibleActions.sort(new Comparator<Action>() {
      @Override
      public int compare(Action a1, Action a2) {
        int a1x = a1.getCoord().getX();
        int a1y = a1.getCoord().getY();
        int a2x = a2.getCoord().getX();
        int a2y = a2.getCoord().getY();

        int numPackages1 = state.getBoard().get(a1x).get(a1y).getNumPackages();
        int numPackages2 = state.getBoard().get(a2x).get(a2y).getNumPackages();
        return numPackages2 - numPackages1;
      }
    });

    List<Action> history = state.getHistory();
    Action lastAction = null;
    if(history.size() > 0) {
      lastAction = history.get(history.size() - 1);
    }

    int opt = optSoFar;
    for(Action action: eligibleActions) {
      // filter actions so that drones are placed with increasing
      // x values
      // if(lastAction != null && action.getCoord().getX() <= lastAction.getCoord().getX()) {
      //   continue;
      // }

      State nextState = problem.step(state, action);
      int resultForThisAction = helper(nextState, opt);
      if(resultForThisAction > opt) {
        opt = resultForThisAction;

        // current result reaches optimal condition in this state
        if(state.getNumPackagesCovered() + state.getNumEligiblePackages() == opt) {
          break;
        }
      }
    }
    return opt;
  }
}

class AstarSolver extends Solver {
  public AstarSolver(Problem problem) {
    super(problem);
  }

  public int solve() {
    State initState = problem.getInitState();
    Comparator<State> stateComparator = new Comparator<State>() {
      @Override
      public int compare(State s1, State s2) {
        int f1 = s1.getNumPackagesCovered() + heuristic(s1);
        int f2 = s2.getNumPackagesCovered() + heuristic(s2);
        return f2 - f1;
      }
    };
    PriorityQueue<State> frontier = new PriorityQueue<>(stateComparator);
    frontier.add(initState);
    Set<State> explored = new HashSet<State>();
    while(true) {
      if(frontier.isEmpty()) {
        // failure
        return 0;
      }
      State state = frontier.poll();
      // super.debug(state);
      if(state.getNumDronesLeft() == 0) {
        return state.getNumPackagesCovered();
      }
      explored.add(state);
      for(Action action : problem.actions(state)) {
        State nextState = problem.step(state, action);
        if((!explored.contains(nextState)) && (!frontier.contains(nextState))) {
          frontier.add(nextState);
        }
      }
    }
  }

  private int heuristic(State state) {
    return state.getNumEligiblePackages();
  }
}

class State {
  private List<List<Cell>> board;
  private List<Action> history;
  private int numDronesPlaced;
  private int numDronesLeft;

  public State(int boardSize, int droneCount, List<Coordinate> packageLocations) {
    board = new ArrayList<>();
    for (int i = 0; i < boardSize; i++) {
      board.add(new ArrayList<Cell>());
      for (int j = 0; j < boardSize; j++) {
        board.get(i).add(new Cell());
      }
    }

    history = new ArrayList<>();
    numDronesPlaced = 0;
    numDronesLeft = droneCount;

    for (Coordinate coord : packageLocations) {
      board.get(coord.getX()).get(coord.getY()).addPackage();
    }
  }

  public State(List<List<Cell>> board, List<Action> history, int numDronesPlaced, int numDronesLeft) {
    int boardSize = board.size();
    this.board = new ArrayList<>();
    for (int i = 0; i < boardSize; i++) {
      this.board.add(new ArrayList<>());
      for (int j = 0; j < boardSize; j++) {
        this.board.get(i).add(new Cell(board.get(i).get(j)));
      }
    }
    this.history = new ArrayList<>();
    for (int i = 0; i < history.size(); i++) {
      this.history.add(new Action(history.get(i)));
    }
    this.numDronesPlaced = numDronesPlaced;
    this.numDronesLeft = numDronesLeft;
  }

  public State(State other) {
    this(other.getBoard(), other.getHistory(), other.getNumDronesPlaced(), other.getNumDronesLeft());
  }

  public List<List<Cell>> getBoard() {
    return board;
  }

  public List<Action> getHistory() {
    return history;
  }

  public int getCost() {
    return history.size();
  }

  public int getNumDronesPlaced() {
    return numDronesPlaced;
  }

  public int getNumDronesLeft() {
    return numDronesLeft;
  }

  public int getNumPackagesCovered() {
    int boardSize = board.size();
    int numCovered = 0;
    for(int i = 0; i < boardSize; i++) {
      for(int j = 0; j < boardSize; j++) {
        Cell cell = board.get(i).get(j);
        if(cell.getDronePlaced()) {
          numCovered += cell.getNumPackages();
        }
      }
    }
    return numCovered;
  }

  private int getNumEligiblePackages(String variation) {
    int boardSize = board.size();

    PriorityQueue<Integer> maxHeap = new PriorityQueue<>(new Comparator<Integer>() {
      @Override
      public int compare(Integer o1, Integer o2) {
        return o2.compareTo(o1);
      }
    });

    if(variation.equals("row")) {
      // count eligible packages when considering one-per-row constraint
      for(int i = 0; i < boardSize; i++) {
        int maxPackageCount = 0;
        boolean dronePlaced = false;
        for(int j = 0; j < boardSize; j++) {
          Cell cell = board.get(i).get(j);
          if((!cell.getCovered()) && (cell.getNumPackages() > maxPackageCount)) {
            maxPackageCount = cell.getNumPackages();
          }
          if(cell.getDronePlaced()) {
            dronePlaced = true;
            break;
          }
        }
        if(!dronePlaced) {
          maxHeap.add(maxPackageCount);
        }
      }
    }
    else if(variation.equals("col")) {
      for(int j = 0; j < boardSize; j++) {
        int maxPackageCount = 0;
        boolean dronePlaced = false;
        for(int i = 0; i < boardSize; i++) {
          Cell cell = board.get(i).get(j);
          if((!cell.getCovered()) && (cell.getNumPackages() > maxPackageCount)) {
            maxPackageCount = cell.getNumPackages();
          }
          if(cell.getDronePlaced()) {
            dronePlaced = true;
            break;
          }
        }
        if(!dronePlaced) {
          maxHeap.add(maxPackageCount);
        }
      }
    }
    else if(variation.equals("diag1")) {
      for(int dist = -(boardSize - 1); dist <= boardSize - 1; dist++) {
        int maxPackageCount = 0;
        boolean dronePlaced = false;
        int iMin = Math.max(0, dist);
        int iMax = Math.min(boardSize - 1, dist + boardSize - 1);
        for(int i = iMin; i <= iMax; i++) {
          int j = i - dist;
          Cell cell = board.get(i).get(j);
          if((!cell.getCovered()) && (cell.getNumPackages() > maxPackageCount)) {
            maxPackageCount = cell.getNumPackages();
          }
          if(cell.getDronePlaced()) {
            dronePlaced = true;
            break;
          }
        }
        if(!dronePlaced) {
          maxHeap.add(maxPackageCount);
        }
      }
    }
    else if(variation.equals("diag2")) {
      for(int sum = 0; sum <= 2 * (boardSize - 1); sum++) {
        int maxPackageCount = 0;
        boolean dronePlaced = false;
        int iMin = Math.max(0, sum - (boardSize - 1));
        int iMax = Math.min(boardSize - 1, sum);
        for(int i = iMin; i <= iMax; i++) {
          int j = sum - i;
          Cell cell = board.get(i).get(j);
          if((!cell.getCovered()) && (cell.getNumPackages() > maxPackageCount)) {
            maxPackageCount = cell.getNumPackages();
          }
          if(cell.getDronePlaced()) {
            dronePlaced = true;
            break;
          }
        }
        if(!dronePlaced) {
          maxHeap.add(maxPackageCount);
        }
      }
    }

    int res = 0;
    for(int i = 0; i < numDronesLeft; i++) {
      res += maxHeap.poll();
    }
    return res;
  }

  public int getNumEligiblePackages() {
    return Math.min(
             Math.min(getNumEligiblePackages("row"),
                      getNumEligiblePackages("col")),
             Math.min(getNumEligiblePackages("diag1"),
                      getNumEligiblePackages("diag2"))
           );
  }

  public int getNumEligiblePackagesArchived() {
    int boardSize = board.size();
    ArrayList<Integer> eligiblePackages = new ArrayList<>();
    for(int i = 0; i < boardSize; i++) {
      for(int j = 0; j < boardSize; j++) {
        Cell cell = board.get(i).get(j);
        if(!cell.getCovered()) {
          eligiblePackages.add(cell.getNumPackages());
        }
      }
    }
    Collections.sort(eligiblePackages);
    int numEligible = 0;
    for(int i = 0; i < Math.min(numDronesLeft, eligiblePackages.size()); i++) {
      numEligible += eligiblePackages.get(eligiblePackages.size() - i - 1);
    }
    return numEligible;
  }

  public void step(Action action) {
    int boardSize = board.size();
    int x = action.getCoord().getX();
    int y = action.getCoord().getY();

    // add drone placement
    board.get(x).get(y).setDronePlaced();

    // add coverage in row
    for (int j = 0; j < boardSize; j++) {
      board.get(x).get(j).setCovered();
    }

    // add coverage in column
    for (int i = 0; i < boardSize; i++) {
      board.get(i).get(y).setCovered();
    }

    // add coverage in primary diagonal
    for (int i = 0; i < boardSize; i++) {
      int j = i + y - x;
      if (j >= 0 && j < boardSize) {
        board.get(i).get(j).setCovered();
      }
    }

    // add coverage in secondary diagonal
    for (int i = 0; i < boardSize; i++) {
      int j = x + y - i;
      if (j >= 0 && j < boardSize) {
        board.get(i).get(j).setCovered();
      }
    }

    history.add(action);
    numDronesLeft--;
    numDronesPlaced++;
  }

  @Override
  public int hashCode() {
    List<String> historyStr = history.stream()
                                     .map((Action ac) -> ("(" + ac.getCoord().getX() + "," + ac.getCoord().getY() + ")"))
                                     .collect(Collectors.toList());
    Collections.sort(historyStr);
    return String.join("", historyStr).hashCode();
  }

  @Override
  public boolean equals(Object obj) {
    if(this == obj) return true;
    if(obj == null) return false;
    if(getClass() != obj.getClass()) return false;
    State other = (State) obj;
    return this.hashCode() == other.hashCode();
  }

  public String toString() {
    StringBuilder sb = new StringBuilder();
    sb.append(String.format("Placed: %3d\n", numDronesPlaced));
    sb.append(String.format("Left  : %3d\n", numDronesLeft));
    sb.append(String.format("Score : %3d\n", getNumPackagesCovered()));
    sb.append(String.format("Eligib: %3d\n", getNumEligiblePackages()));
    for(int i = 0; i < board.size(); i++) {
      sb.append(separatorString() + "\n");
      sb.append(rowString(i) + "\n");
    }
    sb.append(separatorString() + "\n");
    return sb.toString();
  }

  private String separatorString() {
    // returns ascii art for a row separator
    StringBuilder sb = new StringBuilder();
    for(int i = 0; i < board.size(); i++) {
      sb.append("+----");
    }
    sb.append("+");
    return sb.toString();
  }

  private String rowString(int i) {
    // returns ascii art for a row
    StringBuilder sb = new StringBuilder();
    for(int j = 0; j < board.size(); j++) {
      Cell cell = board.get(i).get(j);
      int numPackages = cell.getNumPackages();
      String droneMark = cell.getDronePlaced() ? "x" : " ";
      String coverMark = (cell.getCovered() && numPackages > 0) ? "+" : " ";
      String packageMark = (numPackages > 0) ? String.format("%2d", numPackages) : "  ";
      sb.append(String.format("|%s%s%s", droneMark, coverMark, packageMark));
    }
    sb.append("|");
    return sb.toString();
  }
}

class Cell {
  private int numPackages;
  private boolean covered;
  private boolean dronePlaced;

  public Cell() {
    numPackages = 0;
    covered = false;
    dronePlaced = false;
  }

  public Cell(int numPackages, boolean covered, boolean dronePlaced) {
    this.numPackages = numPackages;
    this.covered = covered;
    this.dronePlaced = dronePlaced;
  }

  public Cell(Cell other) {
    this(other.getNumPackages(), other.getCovered(), other.getDronePlaced());
  }

  public int getNumPackages() {
    return numPackages;
  }

  public boolean getCovered() {
    return covered;
  }

  public boolean getDronePlaced() {
    return dronePlaced;
  }

  public void addPackage() {
    numPackages += 1;
  }

  public void setCovered() {
    covered = true;
  }

  public void setDronePlaced() {
    dronePlaced = true;
  }
}

class Action {
  private Coordinate coord;

  public Action(int x, int y) {
    this.coord = new Coordinate(x, y);
  }

  public Action(Action other) {
    this.coord = new Coordinate(other.getCoord().getX(), other.getCoord().getY());
  }

  public Coordinate getCoord() {
    return coord;
  }
}

class Coordinate {
  private int x;
  private int y;

  public Coordinate(int x, int y) {
    this.x = x;
    this.y = y;
  }

  public int getX() {
    return x;
  }

  public int getY() {
    return y;
  }
}

class project1cs360s2020 {
  public static void main(String[] args) {
    Scanner input = new Scanner(System.in);

    int n = input.nextInt();
    int d = input.nextInt();
    int p = input.nextInt();

    input.nextLine(); // read past endline

    String algorithm = input.nextLine().trim();

    List<Coordinate> coords = new ArrayList<>();
    for (int i = 0; i < p; i++) {
      String coordStr = input.nextLine().trim();
      String[] xy = coordStr.split(",");
      int x = Integer.parseInt(xy[0]);
      int y = Integer.parseInt(xy[1]);
      coords.add(new Coordinate(x, y));
    }

    input.close();

    Problem problem = new Problem(new State(n, d, coords), algorithm);

    System.out.println(problem.solve(algorithm));
  }
}