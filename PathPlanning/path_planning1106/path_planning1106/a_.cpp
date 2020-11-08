#include "a_.h"
using namespace std;

void A2D::Set_scale(int row, int col)
{
    _row = row, _col = col;
    _array = vector<vector<int>>(_row, vector<int>(_col, 0));
}

int A2D::Get_row() const
{
    return _row;
}

int A2D::Get_col() const
{
    return _col;
}

vector<int> A2D::Get_start() const
{
    vector<int> start_point = { _start_x, _start_y };
    return start_point;
}

vector<int> A2D::Get_end() const
{
    vector<int> end_point = { _end_x, _end_y };
    return end_point;
}

void A2D::Set_start(int x, int y)
{
    _start_x = x, _start_y = y;
}

void A2D::Set_end(int x, int y)
{
    _end_x = x, _end_y = y;
}

void A2D::Set_obstacle(int x, int y)
{
    _array[x][y] = OBSTACLE;
}

void A2D::Set_obstacle(vector<int> X, vector<int> Y)
{
    for (int i = 0; i < X.size(); i++)
    {
        _array[X[i]][Y[i]] = OBSTACLE;
    }
}

void A2D::Init_array(vector<int> X, vector<int> Y)
{
    Set_obstacle(X, Y);
    _array[_start_x][_start_y] = START;
    _array[_end_x][_end_y] = END;
}

void A2D::View_array()
{
    for (int i = 0; i < _row; i++)
    {
        for (int j = 0; j < _col; j++)
        {// setiosflags(ios::left) 是输出左对齐的意思！setw(5)指定每个输出的字符占5位
            cout << setiosflags(ios::left) << setw(5) << _array[i][j] << " ";
        }
        cout << endl;
    }
}

void A2D::View_map()
{
    for (int i = 0; i < _row; i++)
    {
        for (int j = 0; j < _col; j++)
        {
            cout << setiosflags(ios::left) << setw(5) << _map[i][j]._data << " ";
        }
        cout << endl;
    }
}

LinkList A2D::InitList()
{
    LinkList L = (LinkList)malloc(sizeof(LNode));
    memset(L,0,sizeof(LNode));
    return L;
}

vector<vector<LNode>> A2D::Creat_A_Map()
{
    for (int i = 0; i < _row; i++)
    {
        vector<LNode> temp;
        for (int j = 0; j < _col; j++)
        {
           LNode p;
           p._data = _array[i][j];
           p._G = 0;
           p._H = 0;
           p._F = 0;
           p._x = i;
           p._y = j;
           p.Close_flag = 0;
           p.OPen_flag = 0;
           p.next = nullptr;
           p.path_next = NULL;
           temp.push_back(p);
        }
        _map.push_back(temp);
    }
    return _map;
}

void A2D::Creat_A_Map_void()
{
    for (int i = 0; i < _row; i++)
    {
        vector<LNode> temp;
        for (int j = 0; j < _col; j++)
        {
           LNode p;
           p._data = _array[i][j];
           p._G = 0;
           p._H = 0;
           p._F = 0;
           p._x = i;
           p._y = j;
           p.Close_flag = 0;
           p.OPen_flag = 0;
           p.next = nullptr;
           p.path_next = NULL;
           temp.push_back(p);
        }
        _map.push_back(temp);
    }
}

bool A2D::isin(int x, int y)
{
    if (x >= 0 && x < _row && y >= 0 && y < _col) return true;
    return false;
}

void A2D::find_start_LNode()
{
    for (int i = 0; i < _row; i++)
    {
        for (int j = 0; j < _col; j++)
        {
            if (_map[i][j]._data == START)
            {
                start_LNode = &_map[i][j];
                start_LNode->_G = 0;
                start_LNode->_H = 0;
                start_LNode->_F = 0;
            }
        }
    }
}

LNode* A2D::Get_start_LNode()
{
    return start_LNode;
}

void A2D::find_end_LNode()
{
    for (int i = 0; i < _row; i++)
    {
        for (int j = 0; j < _col; j++)
        {
            if (_map[i][j]._data == END)
            {
                end_LNode = &_map[i][j];
                end_LNode->_F = 0;
                end_LNode->_G = 0;
                end_LNode->_H = 0;
            }
        }
    }
}

LNode* A2D::Get_end_LNode()
{
    return end_LNode;
}

int A2D::count_LNode_G(LNode* curLNode, LNode* aheadLNode)
{
    if (curLNode->_x == aheadLNode->_x && curLNode->_y == aheadLNode->_y)
    {
        return 0;
    }
    if (aheadLNode->_x - curLNode->_x != 0 && aheadLNode->_y - curLNode->_y != 0)
    {
        curLNode->_G = aheadLNode->_G + 14;
    }
    else curLNode->_G = aheadLNode->_G + 10;
    return curLNode->_G;
}

int A2D::count_LNode_H(LNode *curLNode, LNode *endLNode)
{
    curLNode->_H = abs(endLNode->_x - curLNode->_x) * 10 + abs(endLNode->_y - curLNode->_y) * 10;
    return curLNode->_H;
}

int A2D::count_LNode_F(LNode *curLNode)
{
    curLNode->_F = curLNode->_G + curLNode->_H;
    return curLNode->_F;
}

void A2D::push_OpenList_Node(LinkList L, LNode *elem)
{
    LNode *p, *q;
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

LNode* A2D::pop_OpenList_minNode(LinkList L_OpenList)
{
    LNode *elem = nullptr;
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

bool A2D::insert_Into_CloseList(LNode *min_Open, LinkList L_CloseList)
{
    min_Open->next = L_CloseList->next;
    L_CloseList->next = min_Open;
    min_Open->Close_flag = 1;
    return true;
}

bool A2D::isExist_openList(LNode *curLNode)
{
    return curLNode->OPen_flag;
}

bool A2D::isExist_closeList(LNode *curLNode)
{
    return curLNode->Close_flag;
}

bool A2D::isobstacle(LNode *curLNode)
{
    if (curLNode->_data == OBSTACLE) return true;
    return false;
}

bool A2D::isJoin(LNode *cur)
{
    if (cur->_x > -1 && cur->_y > -1 && cur->_x < _row && cur->_y < _col)
    {
        if (!isExist_closeList(cur) && !isobstacle(cur))
        {
            return true;
        }
        else return false;
    }
    return false;
}

void A2D::insert_open(LNode *Node, LNode *ahead, LNode *endLNode, LinkList open_list)
{
    if (isJoin(Node))
    {
        if (isExist_openList(Node))
        {
            if (Node->_x - ahead->_x != 0 && Node->_y - ahead->_y != 0) {
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

void A2D::check_around_curNode(LNode* cur, LNode* endLNode, LinkList open_list)
{
    int x = cur->_x;
    int y = cur->_y;
    if (isin(x,y-1)) insert_open(&_map[x][y-1],cur,endLNode,open_list);
    if (isin(x,y+1)) insert_open(&_map[x][y+1],cur,endLNode,open_list);
    if (isin(x+1,y)) insert_open(&_map[x+1][y],cur,endLNode,open_list);
    if (isin(x+1,y-1)) insert_open(&_map[x+1][y-1],cur,endLNode,open_list);
    if (isin(x+1,y+1)) insert_open(&_map[x+1][y+1],cur,endLNode,open_list);
    if (isin(x-1,y)) insert_open(&_map[x-1][y],cur,endLNode,open_list);
    if (isin(x-1,y+1)) insert_open(&_map[x-1][y+1],cur,endLNode,open_list);
    if (isin(x-1,y-1)) insert_open(&_map[x-1][y-1],cur,endLNode,open_list);
}




















