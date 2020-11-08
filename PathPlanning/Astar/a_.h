#ifndef A__H
#define A__H

#include <bits/stdc++.h>
using namespace std;

#define BLANK 0
#define OBSTACLE 2
#define START 1
#define END 3

// 用结构体描述A星算法中的一个点!
typedef struct LNode {
	int _data;
	// A星算法中的参数
	int _F;
	int _G;
	int _H;
	// 对应数组中的坐标
	int _x;
	int _y;
	// 开放列表标志位 1表示在开放列表，0表示不在
	bool OPen_flag;
	// 关闭列表标志位 1表示在关闭列表中，0表示不在
	bool Close_flag;
	struct LNode* next;
	struct LNode* path_next;
}LNode, *LinkList;

class A2D {

	public:
		A2D(int row, int col) : _row(row), _col(col) {}

		~A2D();

		// 设置地图的大小
		void Set_scale(int row, int col);
		// 获取地图的行数
		int Get_row() const;
		// 获取地图的列数
		int Get_col() const;
		// 获取起点
		vector<int> Get_start() const;
		// 获取终点
		vector<int> Get_end() const;
		// 设置起点
		void Set_start(int x, int y);
		// 设置终点
		void Set_end(int x, int y);
		// 设置障碍
		void Set_obstacle(int x, int y);

		void Set_obstacle(vector<int> X, vector<int> Y);
		// 初始化地图数组
		void Init_array(vector<int> X, vector<int> Y);
		// 显示数组地图信息
		void View_array();
		// 显示A星地图信息
		void View_map();

		// 初始化节点的链表，返回一个初始化的链表！
		LinkList InitList();

		// 创建A星地图
		vector<vector<LNode>> Creat_A_Map();
		void Creat_A_Map_void();

		// 边界检测
		bool isin(int x, int y);

		// 查找A星地图起点的位置!
		void find_start_LNode();
		// 获取A星地图起点的位置
		LNode* Get_start_LNode();

		// 查找A星地图终点的位置!
		void find_end_LNode();
		// 获取A星地图终点的位置
		LNode* Get_end_LNode();

		// 计算当前节点到上一个节点的G值
		int count_LNode_G(LNode* curLNode, LNode* aheadLNode);

		// 计算当前节点到终点的H值
		int count_LNode_H(LNode* curLNode, LNode* endLNode);

		// 计算F
		int count_LNode_F(LNode* curLNode);

		// 按F值由小到大插入节点到开放列表!
		void push_OpenList_Node(LinkList L, LNode *elem);

		// 取出开放列表中F值最小的节点
		LNode* pop_OpenList_minNode(LinkList L_OpenList);

		// 头插法把用过的点插入关闭列表中
		bool insert_Into_CloseList(LNode* min_Open, LinkList L_CloseList);
		// 查看当前节点是否在开放列表中
		bool isExist_openList(LNode* curLNode);

		// 查看当前节点是否在关闭列表中
		bool isExist_closeList(LNode* curLNode);

		// 判断是否为障碍物
		bool isobstacle(LNode* curLNode);

		// 判断是否应该加入到开放列表中
		bool isJoin(LNode* cur);

		// 插入节点并更新相邻节点间的连接关系
		void insert_open(LNode* Node, LNode* ahead, LNode* endLNode, LinkList open_list);

		// 广度优先搜索
		void check_around_curNode(LNode* cur, LNode* endLNode, LinkList open_list);
	
	public:
		// 地图的长
		int _row = 0;
		// 地图的宽
		int _col = 0;
		// 地图数组
		vector<vector<int>> _array;

		// A星地图
		vector<vector<LNode>> _map;
		// 起点
		int _start_x = 0, _start_y = 0;
		// 终点
		int _end_x = 0, _end_y = 0;

		// A星地图的起始节点
		LNode* start_LNode = NULL;
		// A星地图的最终节点
		LNode* end_LNode = NULL;
};






#endif
