import sys
import math
from nPuzzle.boardState import Node
import heapq
import time

__author__ = 'Naman Kothari'
"""
Author: Naman Kothari(namankothari25@gmail.com)

This program implements different search algorithms
to solve n-puzzle problem.
"""


def getStartBoard(configString):
    """
    Generates a matrix to store the configuration
    of the board
    :param configString: Board configuration in string format
    :return: Board configuration in a matrix
    """

    elements = configString.split(',')

    boardSize = int(math.sqrt(len(elements)))
    board = []

    index = 0

    for i in range(boardSize):
        boardRow = []
        for j in range(boardSize):
            boardRow.append(int(elements[index]))
            index += 1
        board.append(boardRow)

    return board, boardSize

def getEndBoard(boardSize):
    """

    :param boardSize: size of the board (n)
    :return: goal configuration in matrix
    """
    board = []
    index = 0

    for i in range(boardSize):
        boardRow = []
        for j in range(boardSize):
            boardRow.append(index)
            index += 1
        board.append(boardRow)

    return board

def performBFS(startState, endState):
    """
    Finds a solution if any, by applying Breadth-first Search technique
    :param startState: start configuration of the board
    :param endState: end configuration of the board
    :return: boolean representing if the solution is found
    """
    frontier = [startState]
    frontierSet = set()
    visited = set()

    nodesExpanded = 0
    fringeSize = 1
    maxFringe = 1
    maxDepth = 0

    while len(frontier) > 0:
        currentState = frontier.pop(0)
        fringeSize -= 1
        visited.add(str(currentState.getBoard()))

        if isGoal(currentState.getBoard(), endState.getBoard()):

            path = getPath(currentState)
            print('path_to_goal: ' + str(path[::-1]))
            print('cost_of_path: ' + str(len(path)))
            print('nodes_expanded: ' + str(nodesExpanded))
            print('fringe_size: ' + str(fringeSize))
            print('max_fringe_size: ' + str(maxFringe))
            print('search_depth: ' + str(currentState.depth))
            print('max_search_depth: ' + str(maxDepth))

            return True

        neighbours = currentState.getNeighbours()
        nodesExpanded += 1

        for neighbour in neighbours:

            if str(neighbour.getBoard()) not in visited and str(neighbour.getBoard()) not in frontierSet:
                frontier.append(neighbour)
                frontierSet.add(str(neighbour.getBoard()))
                fringeSize += 1

                if maxDepth < neighbour.depth:
                    maxDepth = neighbour.depth

        if fringeSize > maxFringe:
            maxFringe = fringeSize

    print('There is no solution!!')
    return False

def performDFS(startState, endState):
    """
    Finds a solution by applying Depth-first Search technique
    :param startState: start configuration of the board
    :param endState: end configuration of the board
    :return: boolean representing if the solution is found
    """

    frontier = [startState]

    frontierSet = set()
    visited = set()

    nodesExpanded = 0
    fringeSize = 1
    maxFringe = 1
    maxDepth = 0

    while len(frontier) > 0:
        currentState = frontier.pop(-1)
        fringeSize -= 1
        visited.add(str(currentState.getBoard()))

        if isGoal(currentState.getBoard(), endState.getBoard()):

            path = getPath(currentState)
            print('path_to_goal: ' + str(path[::-1]))
            print('cost_of_path: ' + str(len(path)))
            print('nodes_expanded: ' + str(nodesExpanded))
            print('fringe_size: ' + str(fringeSize))
            print('max_fringe_size: ' + str(maxFringe))
            print('search_depth: ' + str(currentState.depth))
            print('max_search_depth: ' + str(maxDepth))

            return True

        neighbours = currentState.getNeighbours()
        neighbours = neighbours[::-1]
        nodesExpanded += 1

        for neighbour in neighbours:

            if str(neighbour.getBoard()) not in visited and str(neighbour.getBoard()) not in frontierSet:
                frontier.append(neighbour)
                frontierSet.add(str(neighbour.getBoard()))
                fringeSize += 1

                if maxDepth < neighbour.depth:
                    maxDepth = neighbour.depth

        if fringeSize > maxFringe:
            maxFringe = fringeSize

    print('There is no solution')
    return False

def performAstar(startState, endState):
    """
    Finds a solution if any, by applying A* Search technique
    :param startState: start configuration of the board
    :param endState: end configuration of the board
    :return: boolean representing if the solution is found
    """
    frontier = []
    heapq.heappush(frontier, startState)
    startState.calculateHeuristic()

    frontierSet = set()
    visited = set()

    nodesExpanded = 0
    fringeSize = 1
    maxFringe = 1
    maxDepth = 0

    while frontier:

        currentState = heapq.heappop(frontier)

        fringeSize -= 1
        visited.add(str(currentState.getBoard()))

        if isGoal(currentState.getBoard(), endState.getBoard()):

            path = getPath(currentState)
            print('path_to_goal: ' + str(path[::-1]))
            print('cost_of_path: ' + str(len(path)))
            print('nodes_expanded: ' + str(nodesExpanded))
            print('fringe_size: ' + str(fringeSize))
            print('max_fringe_size: ' + str(maxFringe))
            print('search_depth: ' + str(currentState.depth))
            print('max_search_depth: ' + str(maxDepth))

            return True

        neighbours = currentState.getNeighbours()
        nodesExpanded += 1

        for neighbour in neighbours:

            if str(neighbour.getBoard()) not in visited and str(neighbour.getBoard()) not in frontierSet:
                neighbour.calculateHeuristic()
                heapq.heappush(frontier, neighbour)
                frontierSet.add(str(neighbour.getBoard()))
                fringeSize += 1

                if maxDepth < neighbour.depth:
                    maxDepth = neighbour.depth

        if fringeSize > maxFringe:
            maxFringe = fringeSize

    print('There is no solution!!!')
    return False

def performIDAstar(startState, endState):
    """
    Finds a solution by applying IDA* Search technique
    :param startState: start configuration of the board
    :param endState: end configuration of the board
    :return: boolean representing if the solution is found
    """
    nodesExpanded = 0
    maxFringe = 1
    maxDepth = 0

    startState.calculateHeuristic()
    costLimit = startState.h * 0.1
    delta = startState.h * 0.05
    costBound = startState.h * 5

    while costLimit < costBound:

        frontier = []
        startState.calculateHeuristic()
        heapq.heappush(frontier, startState)
        startState.calculateHeuristic()

        fringeSize = 1
        frontierSet = set()
        visited = set()

        while frontier:

            currentState = heapq.heappop(frontier)

            if currentState.h > costLimit:
                break

            fringeSize -= 1
            visited.add(str(currentState.getBoard()))

            if isGoal(currentState.getBoard(), endState.getBoard()):

                path = getPath(currentState)
                print('path_to_goal: ' + str(path[::-1]))
                print('cost_of_path: ' + str(len(path)))
                print('nodes_expanded: ' + str(nodesExpanded))
                print('fringe_size: ' + str(fringeSize))
                print('max_fringe_size: ' + str(maxFringe))
                print('search_depth: ' + str(currentState.depth))
                print('max_search_depth: ' + str(maxDepth))

                return True

            neighbours = currentState.getNeighbours()
            nodesExpanded += 1

            for neighbour in neighbours:

                if str(neighbour.getBoard()) not in visited and str(neighbour.getBoard()) not in frontierSet:
                    neighbour.calculateHeuristic()
                    heapq.heappush(frontier, neighbour)
                    frontierSet.add(str(neighbour.getBoard()))
                    fringeSize += 1

                    if maxDepth < neighbour.depth:
                        maxDepth = neighbour.depth

            if fringeSize > maxFringe:
                maxFringe = fringeSize

        costLimit += delta

    print('There is no solution or the solution does not exist in the given cost limit!!')
    return False

def getPath(currentState):
    """
    Find path from start state to current state
    :param currentState: state for which path is to be found
    :return: Path to reach from the start state to current state
    """
    path = []
    parentState = currentState.parent
    while parentState is not None:
        path.append(getStep(parentState, currentState))
        currentState = parentState
        parentState = currentState.parent

    return path

def getStep(parentState, currentState):
    """
    Determines action required to reach from parent to child state
    :param parentState: Parent of the current state
    :param currentState: Child of the parent state
    :return: Action('Up', 'Down, 'Left', 'Right') required to reach from parent to child state
    """
    iParent, jParent = -1, -1
    iCurrent, jCurrent = -1, -1

    parentBoard = parentState.getBoard()
    for i in range(len(parentBoard)):
        for j in range(len(parentBoard)):
            if parentBoard[i][j] == 0:
                iParent, jParent = i, j
                break

    currentBoard = currentState.getBoard()
    for i in range(len(currentBoard)):
        for j in range(len(currentBoard)):
            if currentBoard[i][j] == 0:
                iCurrent, jCurrent = i, j
                break

    if iParent == iCurrent:
        if jParent > jCurrent:
            return 'Left'
        else:
            return 'Right'
    else:
        if iParent > iCurrent:
            return 'Up'
        else:
            return 'Down'

def isGoal(currentBoard, endBoard):
    """
    checks if current state is the end state
    :param currentBoard: board configuration of current state
    :param endBoard: board configuration of end state
    :return: boolean to represent if current state is goal state
    """
    for i in range(len(currentBoard)):
        for j in range(len(currentBoard)):
            if currentBoard[i][j] != endBoard[i][j]:
                return False
    return True

def main():
    """
    Takes input board configuration and search algorithm to find the path
    :return: None
    """
    method, configString = sys.argv[1:]

    startBoard, boardSize = getStartBoard(configString)
    endBoard = getEndBoard(boardSize)

    startState = Node(startBoard, 0)
    endState = Node(endBoard)

    start = time.time()

    if method == 'bfs':
        performBFS(startState, endState)
    elif method == 'dfs':
        performDFS(startState, endState)
    elif method == 'astar':
        performAstar(startState, endState)
    elif method == 'idastar':
        performIDAstar(startState, endState)
    else:
        print('Enter valid method!!')

    end = time.time()

    print('time: ' + str(end - start))

if __name__ == '__main__':
    main()
