# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

BLANK = 0
OBSTACLE = 2
START = 1
END = 3


class LNode:
    def __init__(self):
        self._data = 0
        self._F = 0
        self._G = 0
        self._H = 0
        self._x = 0
        self._y = 0
        self.OPen_flag = False
        self.Close_flag = False
        self.next = None
        self.path_next = None
        
class A2D:
    
    def __init__(self, row, col):
        self._row = row
        self._col = col
        self._array = []
        self._map = []
        self._start_x = 0
        self._start_y = 0
        self._end_x = 0
        self._end_y = 0
        self.start_LNode = LNode()
        self.end_LNode = LNode()
        
    def Set_scale(self, row, col):
        self._row = row
        self._col = col
        self._array = [ [0 for j in range(self._col)] for i in range(self._row) ]
        
    def Get_row(self)->int:
        return self._row
    
    def Get_col(self)->int:
        return self._col
    
    def Get_start(self)->list:
        start_point = [ self._start_x, self._start_y ]
        return start_point
    
    def Get_end(self)->list:
        end_point = [ self._end_x, self._end_y ]
        return end_point
    
    def Set_start(self, x, y):
        self._start_x = x
        self._start_y = y
        
    def Set_end(self, x, y):
        self._end_x = x
        self._end_y = y
     
    
        
    def Set_obstacle(self, X, Y):
        for i in range(len(X)):
            self._array[X[i]][Y[i]] = OBSTACLE
            
    def Init_array(self, X, Y):
        self.Set_obstacle(X, Y)
        self._array[self._start_x][self._start_y] = START
        self._array[self._end_x][self._end_y] = END
        
    def View_array(self):
        for i in range(self._row):
            for j in range(self._col):
                print("%-5d"%self._array[i][j],end="")
            print("\n")
        
    def View_map(self):
        for i in range(self._row):
            for j in range(self._col):
                print("%-5d"%self._map[i][j]._data,end="")
            print("\n")
            
    def InitList(self)->LNode:
        L = LNode()
        return L
    
    def Creat_A_Map_void(self):
        for i in range(self._row):
            temp = []
            for j in range(self._col):
                p = LNode()
                p._data = self._array[i][j]
                p._G = 0
                p._H = 0
                p._F = 0
                p._x = i
                p._y = j
                p.Close_flag = 0
                p.OPen_flag = 0
                p.next = None
                p.path_next = None
                temp.append(p)
            self._map.append(temp)
            
    def isin(self, x, y)->bool:
        if x >= 0 and x < self._row and y >= 0 and y < self._col :
            return True
        return False
    
    def find_start_LNode(self):
        for i in range(self._row):
            for j in range(self._col):
                if self._map[i][j]._data == START:
                    self.start_LNode = self._map[i][j]
                    self.start_LNode._G = 0
                    self.start_LNode._H = 0
                    self.start_LNode._F = 0
                    
    def Get_start_LNode(self)->LNode:
        return self.start_LNode
    
    def find_end_LNode(self):
        for i in range(self._row):
            for j in range(self._col):
                if self._map[i][j]._data == END:
                    self.end_LNode = self._map[i][j]
                    self.end_LNode._F = 0;
                    self.end_LNode._G = 0;
                    self.end_LNode._H = 0;
                    
    def Get_end_LNode(self)->LNode:
        return self.end_LNode
    
    def count_LNode_G(self, curLNode, aheadLNode)->int:
        if curLNode._x == aheadLNode._x and curLNode._y == aheadLNode._y:
            return 0
        if aheadLNode._x - curLNode._x != 0 and aheadLNode._y - curLNode._y != 0:
            curLNode._G = aheadLNode._G + 14
        else:
            curLNode._G = aheadLNode._G + 10
        return curLNode._G
    
    def count_LNode_H(self, curLNode, endLNode)->int:
        curLNode._H = abs(endLNode._x - curLNode._x) * 10 + abs(endLNode._y - curLNode._y) * 10
        return curLNode._H
    
    def count_LNode_F(self, curLNode)->int:
        curLNode._F = curLNode._G + curLNode._H
        return curLNode._F
    
    def push_OpenList_Node(self, L, elem):
        p = L
        q = L
        while p.next != None and p._F < elem._F:
            q = p
            p = p.next
        if p._F < elem._F:
            q = p
        elem.next = q.next
        q.next = elem
        elem.OPen_flag = 1
        
    def pop_OpenList_minNode(self, L_OpenList)->LNode:
        elem = LNode()
        if L_OpenList.next:
            L_OpenList.next.OPen_flag = 0
            elem = L_OpenList.next
            L_OpenList.next = L_OpenList.next.next
            elem.next = None
        else:
            print("have a NULL point in pop_OpenList_minNode()")
        return elem
    
    def insert_Into_CloseList(self, min_Open, L_CloseList):
        min_Open.next = L_CloseList.next
        L_CloseList.next = min_Open
        min_Open.Close_flag = 1
        return True
    
    def isExist_openList(self, curLNode)->bool:
        return curLNode.OPen_flag
    
    def isExist_closeList(self, curLNode)->bool:
        return curLNode.Close_flag
    
    def isobstacle(self, curLNode)->bool:
        if curLNode._data == OBSTACLE:
            return True
        return False
        
    def isJoin(self, cur)->bool:
        if cur._x > -1 and cur._y > -1 and cur._x < self._row and cur._y < self._col:
            if self.isExist_closeList(cur) == False and self.isobstacle(cur) == False:
                return True
            else:
                return False
        return False
    
    def insert_open(self, Node, ahead, endLNode, open_list):
        if self.isJoin(Node):
            if self.isExist_openList(Node):
                if Node._x - ahead._x != 0 and Node._y - ahead._y != 0:
                    if Node._F > (ahead._F + 14):
                        self.count_LNode_G(Node, ahead)
                        self.count_LNode_F(Node)
                        Node.path_next = ahead
                else:
                    if Node._F > (ahead._F + 10):
                        self.count_LNode_G(Node, ahead)
                        self.count_LNode_F(Node)
                        Node.path_next = ahead
            else:
                self.count_LNode_G(Node, ahead)
                self.count_LNode_H(Node, endLNode)
                self.count_LNode_F(Node)
                Node.path_next = ahead
                self.push_OpenList_Node(open_list, Node)
                
    def check_around_curNode(self, cur, endLNode, open_list):
        x = cur._x
        y = cur._y
        if self.isin(x,y-1):
            self.insert_open(self._map[x][y-1],cur,endLNode,open_list)
        if self.isin(x,y+1):
            self.insert_open(self._map[x][y+1],cur,endLNode,open_list)
        if self.isin(x+1,y):
            self.insert_open(self._map[x+1][y],cur,endLNode,open_list)
        if self.isin(x+1,y-1):
            self.insert_open(self._map[x+1][y-1],cur,endLNode,open_list)
        if self.isin(x+1,y+1):
            self.insert_open(self._map[x+1][y+1],cur,endLNode,open_list)
        if self.isin(x-1,y):
            self.insert_open(self._map[x-1][y],cur,endLNode,open_list)
        if self.isin(x-1,y+1):
            self.insert_open(self._map[x-1][y+1],cur,endLNode,open_list)
        if self.isin(x-1,y-1):
            self.insert_open(self._map[x-1][y-1],cur,endLNode,open_list)


if __name__ == "__main__":
    solver = A2D(10,10)
    
    X = [3,3,3,4,4,5,6,6,7,8,8,9]
    Y = [2,5,7,4,5,8,6,2,6,7,8,5]
    
    solver.Set_scale(10,10)
    
    solver.Set_start(0,0)
    
    solver.Set_end(9,9)
    
    solver.Set_obstacle(X, Y)
    solver.Init_array(X, Y)
    
    solver.View_array()
    
    print("=========================================")
    
    print("\n")
    
    solver.Creat_A_Map_void()
    solver.View_map()
    
    solver.find_start_LNode()
    
    solver.find_end_LNode()
    
    print("-----------执行算法------------")
    print("\n")
    
    open_List = solver.InitList()
    print("\n")
    
    close_List = solver.InitList()
    
    solver.find_start_LNode()
    startLNode = solver.start_LNode
    
    solver.find_end_LNode()
    endLNode = solver.end_LNode
    
    curLNode = startLNode
    curLNode._G = 0
    solver.count_LNode_H(curLNode, endLNode)
    solver.count_LNode_F(curLNode)
    
    solver.push_OpenList_Node(open_List, curLNode)
    
    while curLNode._data != END:
        curLNode = solver.pop_OpenList_minNode(open_List)
        solver.insert_Into_CloseList(curLNode, close_List)
        solver.check_around_curNode(curLNode, endLNode, open_List)
    
    ptr = endLNode
    path = []
    
    while ptr:
        path.append([ptr._x, ptr._y])
        ptr = ptr.path_next
        
    reversed(path)
    
    for i in range(len(path)):
        print(path[i][0],path[i][1])
        
    for i in range(len(path)):
        solver._array[path[i][0]][path[i][1]] = 7
        
    for i in range(solver._row):
        for j in range(solver._col):
            print("%-5d"%solver._array[i][j],end="")
        print("\n")
    
       
                
                
                
                
                
                
                
                
                
                
                
                
                
        
        
        
        
        
        
        
        
        
        
        
        
        
        