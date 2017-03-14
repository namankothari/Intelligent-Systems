class Node:

    __slots__ = 'board', 'depth', 'neighbours', 'parent', 'f', 'g', 'h'

    def __init__(self, board, depth=-1, parent=None):
        self.board = board
        self.depth = depth
        self.parent = parent
        self.neighbours = []

    def __lt__(self, other):
        return self.f < other.f

    def calculateHeuristic(self):
        if self.parent is not None:
            self.g = self.parent.g + 1
        else:
            self.g = 0
        self.h = self.getManhattan()
        self.f = self.g + self.h

    def getManhattan(self):
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
        return self.board

    def getNeighbours(self):

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
        for i in range(len(self.board)):
            for j in range(len(self.board)):
                if self.board[i][j] == 0:
                    return i, j
