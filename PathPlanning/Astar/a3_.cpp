#include "a3_.h"

void A3D::Set_scale(int row, int col, int height)
{
    _row = row, _col = col, _height = height;
    _array3 = vector<vector<vector<int>>>(_height);
    for (int i = 0; i < _height; i++)
    {
        _array3[i] = vector<vector<int>>(_row, vector<int>(_col, 0));
    }

}

int A3D::Get_row() const
{
    return _row;
}

int A3D::Get_col() const
{
    return _col;
}

vector<int> A3D::Get_start() const
{
    vector<int> start_point = { _start_x, _start_y, _start_z };
    return start_point;
}

vector<int> A3D::Get_end() const
{
    vector<int> end_point = { _end_x, _end_y, _end_z };
    return end_point;
}

void A3D::Set_start(int x, int y, int z)
{
    _start_x = x, _start_y = y, _start_z = z;
}

void A3D::Set_end(int x, int y, int z)
{
    _end_x = x, _end_y = y, _end_z = z;
}

void A3D::Set_obstacle(int x, int y, int z)
{
    _array3[x][y][z] = OBSTACLE;
}

void A3D::Set_obstacle(vector<int> X, vector<int> Y, vector<int> Z)
{
    for (int i = 0; i < X.size(); i++)
    {
        _array3[X[i]][Y[i]][Z[i]] = OBSTACLE;
    }
}

void A3D::Init_array(vector<int> X, vector<int> Y, vector<int> Z)
{
    Set_obstacle(X, Y, Z);
    _array3[_start_x][_start_y][_start_z] = START;
    _array3[_end_x][_end_y][_end_z] = END;
}

void A3D::View_array()
{
    for (int k = 0; k < _height; k++)
    {
        for (int i = 0; i < _row; i++)
        {
            for (int j = 0; j < _col; j++)
            {// setiosflags(ios::left) 是输出左对齐的意思！setw(5)指定每个输出的字符占5位
                cout << setiosflags(ios::left) << setw(5) << _array3[i][j][k] << " ";
            }
            cout << endl;
        }
        cout << endl;
        cout << endl;
    }
}

void A3D::View_map()
{
    for (int k = 0; k < _height; k++)
    {
        for (int i = 0; i < _row; i++)
        {
            for (int j = 0; j < _col; j++)
            {
                cout << setiosflags(ios::left) << setw(5) << _map3[i][j][k]._data << " ";
            }
            cout << endl;
        }
        cout << endl;
        cout << endl;
    }
}

LinkList3 A3D::InitList()
{
    LinkList3 L = (LinkList3)malloc(sizeof(LNode3));
    memset(L,0,sizeof(LNode3));
    return L;
}

vector<vector<vector<LNode3>>> A3D::Creat_A_Map()
{
    for (int k = 0; k < _height; k++)
    {
        vector<vector<LNode3>> tem;
        for (int i = 0; i < _row; i++)
        {
            vector<LNode3> temp;
            for (int j = 0; j < _col; j++)
            {
               LNode3 p;
               p._data = _array3[i][j][k];
               p._G = 0;
               p._H = 0;
               p._F = 0;
               p._x = i;
               p._y = j;
               p._z = k;
               p.Close_flag = 0;
               p.OPen_flag = 0;
               p.next = nullptr;
               p.path_next = NULL;
               temp.push_back(p);
            }
            tem.push_back(temp);
        }
        _map3.push_back(tem);
    }
    return _map3;
}

void A3D::Creat_A_Map_void()
{
    for (int i = 0; i < _row; i++)
    {
        vector<vector<LNode3>> tem;
        for (int j = 0; j < _col; j++)
        {
            vector<LNode3> temp;
            for (int k = 0; k < _height; k++)
            {
                LNode3 p;
                temp.push_back(p);
            }
            tem.push_back(temp);
        }
        _map3.push_back(tem);
    }
    for (int k = 0; k < _height; k++)
    {
        for (int i = 0; i < _row; i++)
        {
            for (int j = 0; j < _col; j++)
            {
               _map3[i][j][k]._data = _array3[i][j][k];
               _map3[i][j][k]._G = 0;
               _map3[i][j][k]._H = 0;
               _map3[i][j][k]._F = 0;
               _map3[i][j][k]._x = i;
               _map3[i][j][k]._y = j;
               _map3[i][j][k]._z = k;
               _map3[i][j][k].Close_flag = 0;
               _map3[i][j][k].OPen_flag = 0;
               _map3[i][j][k].next = nullptr;
               _map3[i][j][k].path_next = NULL;
            }
        }
    }
}

bool A3D::isin(int x, int y, int z)
{
    if (x >= 0 && x < _row && y >= 0 && y < _col && z >= 0 && z < _height) return true;
    return false;
}

void A3D::find_start_LNode()
{
    for (int k = 0; k < _height; k++)
    {
        for (int i = 0; i < _row; i++)
        {
            for (int j = 0; j < _col; j++)
            {
                if (_map3[i][j][k]._data == START)
                {
                    start_LNode3 = &_map3[i][j][k];
                    start_LNode3->_G = 0;
                    start_LNode3->_H = 0;
                    start_LNode3->_F = 0;
                }
            }
        }
    }
}

LNode3* A3D::Get_start_LNode()
{
    return start_LNode3;
}

void A3D::find_end_LNode()
{
    for (int k = 0; k < _height; k++)
    {
        for (int i = 0; i < _row; i++)
        {
            for (int j = 0; j < _col; j++)
            {
                if (_map3[i][j][k]._data == END)
                {
                    end_LNode3 = &_map3[i][j][k];
                    end_LNode3->_F = 0;
                    end_LNode3->_G = 0;
                    end_LNode3->_H = 0;
                }
            }
        }
    }
}

LNode3* A3D::Get_end_LNode()
{
    return end_LNode3;
}

int A3D::count_LNode_G(LNode3* curLNode, LNode3* aheadLNode)
{
    int DIS = abs(curLNode->_x - aheadLNode->_x);
        DIS += abs(curLNode->_y - aheadLNode->_y);
        DIS += abs(curLNode->_z - aheadLNode->_z);
    if (DIS == 0) return 0;
    if (DIS == 1) curLNode->_G = aheadLNode->_G + 10;
    if (DIS == 2) curLNode->_G = aheadLNode->_G + 14;
    if (DIS == 3) curLNode->_G = aheadLNode->_G + 17;
    return curLNode->_G;
}

int A3D::count_LNode_H(LNode3 *curLNode, LNode3 *endLNode3)
{
    curLNode->_H = abs(endLNode3->_x - curLNode->_x) * 10 + abs(endLNode3->_y - curLNode->_y) * 10 + abs(endLNode3->_z - curLNode->_z) * 10;
    return curLNode->_H;
}

int A3D::count_LNode_F(LNode3 *curLNode)
{
    curLNode->_F = curLNode->_G + curLNode->_H;
    return curLNode->_F;
}

void A3D::push_OpenList_Node(LinkList3 L, LNode3 *elem)
{
    LNode3 *p, *q;
    p = q = L;
    while (p->next != NULL && p->_F < elem->_F)
    {
        q = p;
        p = p->next;
    }
    if (p->_F < elem->_F) q = p;
    elem->next = q->next;
    q->next = elem;
    elem->OPen_flag = 1;
}

LNode3* A3D::pop_OpenList_minNode(LinkList3 L_OpenList)
{
    LNode3 *elem = nullptr;
    if (L_OpenList->next)
    {
        L_OpenList->next->OPen_flag = 0;
        elem = L_OpenList->next;
        L_OpenList->next = L_OpenList->next->next;
        elem->next = NULL;
    }
    else printf("have a NULL point in pop_OpenList_mimNode()");
    return elem;
}

bool A3D::insert_Into_CloseList(LNode3 *min_Open, LinkList3 L_CloseList)
{
    min_Open->next = L_CloseList->next;
    L_CloseList->next = min_Open;
    min_Open->Close_flag = 1;
    return true;
}

bool A3D::isExist_openList(LNode3 *curLNode)
{
    return curLNode->OPen_flag;
}

bool A3D::isExist_closeList(LNode3 *curLNode)
{
    return curLNode->Close_flag;
}

bool A3D::isobstacle(LNode3 *curLNode)
{
    if (curLNode->_data == OBSTACLE) return true;
    return false;
}

bool A3D::isJoin(LNode3 *cur)
{
    if (cur->_x > -1 && cur->_y > -1 && cur->_z > -1 && cur->_x < _row && cur->_y < _col && cur->_z < _height)
    {
        if (!isExist_closeList(cur) && !isobstacle(cur))
        {
            return true;
        }
        else return false;
    }
    return false;
}

void A3D::insert_open(LNode3 *Node, LNode3 *ahead, LNode3 *endLNode, LinkList3 open_list)
{
    if (isJoin(Node))
    {
        if (isExist_openList(Node))
        {
            int DIS = abs(Node->_x - ahead->_x) + abs(Node->_y - ahead->_y) + abs(Node->_z - ahead->_z);
            if (DIS == 3)
            {
                if (Node->_F > (ahead->_F + 17))
                {
                    count_LNode_G(Node, ahead);
                    count_LNode_F(Node);
                    Node->path_next = ahead;
                }
            }
            else if (DIS == 2)
            {
                if (Node->_F > (ahead->_F + 14))
                {
                    count_LNode_G(Node, ahead);
                    count_LNode_F(Node);
                    Node->path_next = ahead;
                }
            }
            else
            {
                if (Node->_F > (ahead->_F + 10))
                {
                    count_LNode_G(Node, ahead);
                    count_LNode_F(Node);
                    Node->path_next = ahead;
                }
            }
        }
        else
        {
            count_LNode_G(Node, ahead);
            count_LNode_H(Node, endLNode);
            count_LNode_F(Node);
            Node->path_next = ahead;
            push_OpenList_Node(open_list, Node);
        }
    }
}

void A3D::check_around_curNode(LNode3* cur, LNode3* endLNode, LinkList3 open_list)
{
    int x = cur->_x;
    int y = cur->_y;
    int z = cur->_z;
    if (isin(x,y-1,z)) insert_open(&_map3[x][y-1][z],cur,endLNode,open_list);
    if (isin(x,y+1,z)) insert_open(&_map3[x][y+1][z],cur,endLNode,open_list);
    if (isin(x+1,y,z)) insert_open(&_map3[x+1][y][z],cur,endLNode,open_list);
    if (isin(x+1,y-1,z)) insert_open(&_map3[x+1][y-1][z],cur,endLNode,open_list);
    if (isin(x+1,y+1,z)) insert_open(&_map3[x+1][y+1][z],cur,endLNode,open_list);
    if (isin(x-1,y,z)) insert_open(&_map3[x-1][y][z],cur,endLNode,open_list);
    if (isin(x-1,y+1,z)) insert_open(&_map3[x-1][y+1][z],cur,endLNode,open_list);
    if (isin(x-1,y-1,z)) insert_open(&_map3[x-1][y-1][z],cur,endLNode,open_list);
    if (isin(x,y-1,z-1)) insert_open(&_map3[x][y-1][z-1],cur,endLNode,open_list);
    if (isin(x,y,z-1)) insert_open(&_map3[x][y][z-1],cur,endLNode,open_list);
    if (isin(x,y+1,z-1)) insert_open(&_map3[x][y+1][z-1],cur,endLNode,open_list);
    if (isin(x+1,y,z-1)) insert_open(&_map3[x+1][y][z-1],cur,endLNode,open_list);
    if (isin(x+1,y-1,z-1)) insert_open(&_map3[x+1][y-1][z-1],cur,endLNode,open_list);
    if (isin(x+1,y+1,z-1)) insert_open(&_map3[x+1][y+1][z-1],cur,endLNode,open_list);
    if (isin(x-1,y,z-1)) insert_open(&_map3[x-1][y][z-1],cur,endLNode,open_list);
    if (isin(x-1,y+1,z-1)) insert_open(&_map3[x-1][y+1][z-1],cur,endLNode,open_list);
    if (isin(x-1,y-1,z-1)) insert_open(&_map3[x-1][y-1][z-1],cur,endLNode,open_list);
    if (isin(x,y-1,z+1)) insert_open(&_map3[x][y-1][z+1],cur,endLNode,open_list);
    if (isin(x,y,z+1)) insert_open(&_map3[x][y][z+1],cur,endLNode,open_list);
    if (isin(x,y+1,z+1)) insert_open(&_map3[x][y+1][z+1],cur,endLNode,open_list);
    if (isin(x+1,y,z+1)) insert_open(&_map3[x+1][y][z+1],cur,endLNode,open_list);
    if (isin(x+1,y-1,z+1)) insert_open(&_map3[x+1][y-1][z+1],cur,endLNode,open_list);
    if (isin(x+1,y+1,z+1)) insert_open(&_map3[x+1][y+1][z+1],cur,endLNode,open_list);
    if (isin(x-1,y,z+1)) insert_open(&_map3[x-1][y][z+1],cur,endLNode,open_list);
    if (isin(x-1,y+1,z+1)) insert_open(&_map3[x-1][y+1][z+1],cur,endLNode,open_list);
    if (isin(x-1,y-1,z+1)) insert_open(&_map3[x-1][y-1][z+1],cur,endLNode,open_list);
}
