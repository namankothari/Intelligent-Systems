__author__ = 'Naman Kothari'
"""
Author: Naman Kothari(namankothari25@gmail.com)

This object of this class represents a state in n-puzzle problem.

"""

class Node:
    __slots__ = 'board', 'depth', 'neighbours', 'parent', 'f', 'g', 'h'

    def __init__(self, board, depth=-1, parent=None):
        """

        :param board: matrix to hold the board configuration
        :param depth: depth of the state with respect to start state
        :param parent: parent state from which this state is generated
        """
        self.board = board
        self.depth = depth
        self.parent = parent
        self.neighbours = []

    def __lt__(self, other):
        return self.f < other.f

    def calculateHeuristic(self):
        """
        Computes the value of f, g, h required for A* and IDA*
        :return: None
        """
        if self.parent is not None:
            self.g = self.parent.g + 1
        else:
            self.g = 0
        self.h = self.getManhattan()
        self.f = self.g + self.h

    def getManhattan(self):
        """
        Calculates heuristic cost of current state by using manhattan distance
        :return: heuristic cost
        """
        steps = 0
        for index in range(1, len(self.board)**2):
            iGoal, jGoal = index / len(self.board), index % len(self.board)
            iCurr, jCurr = -1, -1

            for i in range(len(self.board)):
                for j in range(len(self.board)):
                    if self.board[i][j] == index:
                        iCurr, jCurr = i, j
                        break

            steps += abs(iCurr - iGoal) + abs(jCurr - jGoal)

        return steps

    def getBoard(self):
        """

        :return: board configuration of current state
        """
        return self.board

    def getNeighbours(self):
        """
        Generates the neighbours of the current state
        :return: neighbouring states of current state
        """

        iBlank, jBlank = self.getBlankIndex()

        upNeighbour = self.getUpNeighbour(iBlank, jBlank)
        if upNeighbour != -1:
            self.neighbours.append(Node(upNeighbour, self.depth + 1, self))

        downNeighbour = self.getDownNeighbour(iBlank, jBlank)
        if downNeighbour != -1:
            self.neighbours.append(Node(downNeighbour, self.depth + 1, self))

        leftNeighbour = self.getLeftNeighbour(iBlank, jBlank)
        if leftNeighbour != -1:
            self.neighbours.append(Node(leftNeighbour, self.depth + 1, self))

        rightNeighbour = self.getRightNeighbour(iBlank, jBlank)
        if rightNeighbour != -1:
            self.neighbours.append(Node(rightNeighbour, self.depth + 1, self))

        return self.neighbours

    def getLeftNeighbour(self, iBlank, jBlank):
        """

        :param iBlank: row of blank tile
        :param jBlank: col of blank tile
        :return: left neighbour of the current state
        """
        if jBlank == 0:
            return -1

        nextBoard = []

        for i in range(len(self.board)):
            boardRow = []
            for j in range(len(self.board)):
                boardRow.append(self.board[i][j])
            nextBoard.append(boardRow)

        nextBoard[iBlank][jBlank], nextBoard[iBlank][jBlank - 1] = nextBoard[iBlank][jBlank - 1], nextBoard[iBlank][
            jBlank]

        return nextBoard

    def getRightNeighbour(self, iBlank, jBlank):
        """

        :param iBlank: row of blank tile
        :param jBlank: col of blank tile
        :return: right neighbour of the current state
        """
        if jBlank == len(self.board) - 1:
            return -1

        nextBoard = []
        for i in range(len(self.board)):
            boardRow = []
            for j in range(len(self.board)):
                boardRow.append(self.board[i][j])
            nextBoard.append(boardRow)

        nextBoard[iBlank][jBlank], nextBoard[iBlank][jBlank + 1] = nextBoard[iBlank][jBlank + 1], nextBoard[iBlank][
            jBlank]

        return nextBoard

    def getUpNeighbour(self, iBlank, jBlank):
        """

        :param iBlank: row of blank tile
        :param jBlank: col of blank tile
        :return: up neighbour of the current state
        """
        if iBlank == 0:
            return -1

        nextBoard = []
        for i in range(len(self.board)):
            boardRow = []
            for j in range(len(self.board)):
                boardRow.append(self.board[i][j])
            nextBoard.append(boardRow)

        nextBoard[iBlank][jBlank], nextBoard[iBlank - 1][jBlank] = nextBoard[iBlank - 1][jBlank], nextBoard[iBlank][
            jBlank]

        return nextBoard

    def getDownNeighbour(self, iBlank, jBlank):
        """

        :param iBlank: row of blank tile
        :param jBlank: col of blank tile
        :return: down neighbour of the current state
        """
        if iBlank == len(self.board) - 1:
            return -1

        nextBoard = []
        for i in range(len(self.board)):
            boardRow = []
            for j in range(len(self.board)):
                boardRow.append(self.board[i][j])
            nextBoard.append(boardRow)

        nextBoard[iBlank][jBlank], nextBoard[iBlank + 1][jBlank] = nextBoard[iBlank + 1][jBlank], nextBoard[iBlank][
            jBlank]

        return nextBoard

    def getBlankIndex(self):
        """

        :return: row and column of the blank tile
        """
        for i in range(len(self.board)):
            for j in range(len(self.board)):
                if self.board[i][j] == 0:
                    return i, j
