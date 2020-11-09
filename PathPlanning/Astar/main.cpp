#include <bits/stdc++.h>
#include <condition_variable>
#include <thread>
#include <mutex>
#include "a_.h"
#include "a3_.h"
using namespace std;

std::mutex mymutex;
std::condition_variable cv;
void astar2d()
{
    A2D* solver = new A2D(10,10);

    //----------------- 初始化地图数组并显示 -----------------//
    vector<int> X = {3,3,3,4,4,5,6,6,7,8,8,9};
    vector<int> Y = {2,5,7,4,5,8,6,2,6,7,8,5};

    solver->Set_scale(10,10);

    solver->Set_start(0, 0);

    solver->Set_end(9, 9);

    solver->Set_obstacle(X, Y);
    solver->Init_array(X, Y);

    solver->Creat_A_Map_void();

    solver->find_start_LNode();
    solver->find_end_LNode();

    // ---------------- 开始执行算法 ----------------

    // 初始化开放列表
    LinkList open_List = solver->InitList();

    // 初始化关闭列表
    LinkList close_List = solver->InitList();

    // 查找起点
    solver->find_start_LNode();
    LNode* startLNode = solver->start_LNode;

    // 查找终点
    solver->find_end_LNode();
    LNode* endLNode = solver->end_LNode;

    // 初始化当前节点
    LNode* curLNode = startLNode;
    curLNode->_G = 0;
    solver->count_LNode_H(curLNode, endLNode);
    solver->count_LNode_F(curLNode);
    // 先将起点压入开放列表
    solver->push_OpenList_Node(open_List, curLNode);

    while (curLNode->_data != END)
    {
        curLNode = solver->pop_OpenList_minNode(open_List);
        solver->insert_Into_CloseList(curLNode, close_List);
        solver->check_around_curNode(curLNode, endLNode, open_List);
    }

    LNode* ptr = endLNode;
    vector<vector<int>> path;
    while (ptr)
    {
        path.push_back({ptr->_x,ptr->_y});
        ptr = ptr->path_next;
    }
    reverse(path.begin(),path.end());
    // 在二维数组上显示

    for (int i = 0; i < path.size(); i++)
    {
        solver->_array[path[i][0]][path[i][1]] = 7;
    }

    // 终端输出口是公共使用区域存在抢占情况所以要加锁保护
    // 使用std::unique_lock<std::mutex> lg(mymutex);
    // 那个线程先到先使用终端输出，在执行完下面的输出程序后通过 cv.notify_all();唤醒其他所有线程！
    std::unique_lock<std::mutex> lg(mymutex);

    for (int i = 0; i < solver->_row; i++)
    {
        for (int j = 0; j < solver->_col; j++)
        {
            cout << setiosflags(ios::left) << setw(5) << solver->_array[i][j];
        }
        cout << endl;
    }
    cout << endl;
    cv.notify_all();
}

void astar3d()
{
    A3D* solver3 = new A3D(5,5,5);
    solver3->Set_scale(5,5,5);
    solver3->Set_start(0,0,0);
    solver3->Set_end(4,4,4);
    vector<int> xx = {0,0,0,0,0,1,1,1,1,1,1,1,1,1,2,2,2,2,2,3,3,3,3,3,4,4,4,4,4};
    vector<int> yy = {1,1,1,1,1,0,0,0,0,0,1,1,1,1,3,3,3,3,3,3,3,3,3,3,1,1,1,1,1};
    vector<int> zz = {0,1,2,3,4,0,1,2,3,4,0,1,2,3,0,1,2,3,4,0,1,2,3,4,0,1,2,3,4};
    //solver3->Set_obstacle(xx,yy,zz);
    solver3->Init_array(xx,yy,zz);

    solver3->Creat_A_Map_void();


    LinkList3 open_List3 = solver3->InitList();

    LinkList3 close_List3 = solver3->InitList();

    solver3->find_start_LNode();
    LNode3* startLNode3 = solver3->Get_start_LNode();

    solver3->find_end_LNode();
    LNode3* endLNode3 = solver3->Get_end_LNode();

    LNode3* curLNode3 = startLNode3;

    curLNode3->_G = 0;

    solver3->count_LNode_H(curLNode3, endLNode3);

    solver3->count_LNode_F(curLNode3);

    solver3->push_OpenList_Node(open_List3, curLNode3);

    while (curLNode3->_data != 3)
    {
        curLNode3 = solver3->pop_OpenList_minNode(open_List3);
        solver3->insert_Into_CloseList(curLNode3, close_List3);
        solver3->check_around_curNode(curLNode3, endLNode3, open_List3);
    }

    LNode3* p3 = endLNode3;
    vector<vector<int>> path3;
    while (p3)
    {
        path3.push_back({p3->_x, p3->_y, p3->_z});
        p3 = p3->path_next;
    }
    reverse(path3.begin(),path3.end());

    for (int i = 0; i < path3.size(); i++)
    {
        solver3->_array3[path3[i][0]][path3[i][1]][path3[i][2]] = 7;
    }

    // 终端输出口是公共使用区域存在抢占情况所以要加锁保护
    // 使用std::unique_lock<std::mutex> lg(mymutex);
    // 那个线程先到先使用终端输出，在执行完下面的输出程序后通过 cv.notify_all();唤醒其他所有线程！
    std::unique_lock<std::mutex> lg(mymutex);

    // 在二维数组上显示
    for (int k = 0; k < solver3->_height; k++)
    {
        for (int i = 0; i < solver3->_row; i++)
        {
            for (int j = 0; j < solver3->_col; j++)
            {
                cout << setiosflags(ios::left) << setw(5) << solver3->_array3[i][j][k];
            }
            cout << endl;
        }
        cout << endl;
        cout << endl;
    }
    cout << endl;
    cv.notify_all();
}

int main()
{
    /*
    A2D* solver = new A2D(10,10);

    //----------------- 初始化地图数组并显示 -----------------//
    vector<int> X = {3,3,3,4,4,5,6,6,7,8,8,9};
    vector<int> Y = {2,5,7,4,5,8,6,2,6,7,8,5};

    solver->Set_scale(10,10);
    cout << "The row of map is : " << solver->Get_row() << endl;
    cout << "The col of map is : " << solver->Get_col() << endl;

    solver->Set_start(0, 0);
    cout << "The start point of map is : " << "(" << solver->Get_start()[0] << "," << solver->Get_start()[1] << ")" << endl;
    solver->Set_end(9, 9);
    cout << "The end point of map is : " << "(" << solver->Get_end()[0] << "," << solver->Get_end()[1] << ")" << endl;

    solver->Set_obstacle(X, Y);
    solver->Init_array(X, Y);

    solver->View_array();
    //----------------------------------------------------//
    cout << endl;
    solver->Creat_A_Map_void();
    solver->View_map();

    solver->find_start_LNode();
    cout << "The start point of A* map : " << solver->Get_start_LNode()->_data << endl;

    solver->find_end_LNode();
    cout << "The end point of A* map : " << solver->Get_end_LNode()->_data << endl;

    // ---------------- 开始执行算法 ----------------

    // 初始化开放列表
    LinkList open_List = solver->InitList();
    cout << endl;
    cout << "The address of the open_List is : " << open_List << endl;

    // 初始化关闭列表
    LinkList close_List = solver->InitList();
    cout << endl;
    cout << "The address of the close_List is : " << close_List << endl;

    // 查找起点
    solver->find_start_LNode();
    LNode* startLNode = solver->start_LNode;
    cout << endl;
    cout << "The address of the startLNode is : " << solver->start_LNode->_data << endl;

    // 查找终点
    solver->find_end_LNode();
    LNode* endLNode = solver->end_LNode;
    cout << endl;
    cout << "The address of the endLNode is : " << solver->end_LNode->_data << endl;

    // 初始化当前节点
    LNode* curLNode = startLNode;
    curLNode->_G = 0;
    solver->count_LNode_H(curLNode, endLNode);
    solver->count_LNode_F(curLNode);
    // 先将起点压入开放列表
    solver->push_OpenList_Node(open_List, curLNode);

    while (curLNode->_data != END)
    {
        curLNode = solver->pop_OpenList_minNode(open_List);
        solver->insert_Into_CloseList(curLNode, close_List);
        solver->check_around_curNode(curLNode, endLNode, open_List);
    }

    LNode* ptr = endLNode;
    vector<vector<int>> path;
    while (ptr)
    {
        path.push_back({ptr->_x,ptr->_y});
        ptr = ptr->path_next;
    }
    reverse(path.begin(),path.end());
    // 输出路径！
    cout << endl;
    for (int i = 0; i < path.size(); i++)
    {
        cout << "(" << path[i][0] << "," << path[i][1] << ")" << endl;
    }
    // --------------------------------------------

    // 在二维数组上显示
    for (int i = 0; i < path.size(); i++)
    {
        solver->_array[path[i][0]][path[i][1]] = 7;
    }
    for (int i = 0; i < solver->_row; i++)
    {
        for (int j = 0; j < solver->_col; j++)
        {
            cout << setiosflags(ios::left) << setw(5) << solver->_array[i][j];
        }
        cout << endl;
    }

    cout << "========================================================";
    cout << "------------------------ 分隔 ---------------------------";
    cout << endl;
    A3D* solver3 = new A3D(5,5,5);
    solver3->Set_scale(5,5,5);
    solver3->Set_start(0,0,0);
    solver3->Set_end(4,4,4);
    vector<int> xx = {0,0,0,0,0,1,1,1,1,1,1,1,1,1,2,2,2,2,2,3,3,3,3,3,4,4,4,4,4};
    vector<int> yy = {1,1,1,1,1,0,0,0,0,0,1,1,1,1,3,3,3,3,3,3,3,3,3,3,1,1,1,1,1};
    vector<int> zz = {0,1,2,3,4,0,1,2,3,4,0,1,2,3,0,1,2,3,4,0,1,2,3,4,0,1,2,3,4};
    //solver3->Set_obstacle(xx,yy,zz);
    solver3->Init_array(xx,yy,zz);
    solver3->View_array();
    cout << endl;
    cout << "=========================================================" << endl;
    solver3->Creat_A_Map_void();
    solver3->View_map();
    cout << endl;


    LinkList3 open_List3 = solver3->InitList();

    LinkList3 close_List3 = solver3->InitList();

    solver3->find_start_LNode();
    LNode3* startLNode3 = solver3->Get_start_LNode();

    solver3->find_end_LNode();
    LNode3* endLNode3 = solver3->Get_end_LNode();

    LNode3* curLNode3 = startLNode3;

    curLNode3->_G = 0;

    solver3->count_LNode_H(curLNode3, endLNode3);

    solver3->count_LNode_F(curLNode3);

    solver3->push_OpenList_Node(open_List3, curLNode3);

    while (curLNode3->_data != 3)
    {
        curLNode3 = solver3->pop_OpenList_minNode(open_List3);
        solver3->insert_Into_CloseList(curLNode3, close_List3);
        solver3->check_around_curNode(curLNode3, endLNode3, open_List3);
    }

    LNode3* p3 = endLNode3;
    vector<vector<int>> path3;
    while (p3)
    {
        path3.push_back({p3->_x, p3->_y, p3->_z});
        p3 = p3->path_next;
    }
    reverse(path3.begin(),path3.end());
    // 输出路径
    for (int i = 0; i < path3.size(); i++)
    {
        cout << "(" << path3[i][0] << "," << path3[i][1] << "," << path3[i][2] << ")" << endl;
    }

    for (int i = 0; i < path3.size(); i++)
    {
        solver3->_array3[path3[i][0]][path3[i][1]][path3[i][2]] = 7;
    }

    // 在二维数组上显示
    for (int k = 0; k < solver3->_height; k++)
    {
        for (int i = 0; i < solver3->_row; i++)
        {
            for (int j = 0; j < solver3->_col; j++)
            {
                cout << setiosflags(ios::left) << setw(5) << solver3->_array3[i][j][k];
            }
            cout << endl;
        }
        cout << endl;
        cout << endl;
    }
    */


    std::thread A1(astar2d);
    std::thread A2(astar3d);
    A1.join();
    A2.join();

    return 0;
}
