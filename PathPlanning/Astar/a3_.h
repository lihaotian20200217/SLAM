#ifndef A3__H
#define A3__H

#include "a_.h"

typedef struct LNode3 {
	int _data;
	int _F;
	int _G;
	int _H;
	int _x;
	int _y;
	int _z;
	bool OPen_flag;
	bool Close_flag;
	struct LNode3* next;
	struct LNode3* path_next;
}LNode3, *LinkList3;

class A3D : public A2D
{
	public:
		A3D(int row, int col, int height) : A2D(row,col), _height(height) {}
		~A3D();

		// 设置地图大小
		void Set_scale(int row, int col, int height);
		// 获取地图的行数
		int Get_row() const;
		// 获取地图的列数
		int Get_col() const;
		// 获取起点
		vector<int> Get_start() const;
		// 获取终点
		vector<int> Get_end() const;
		// 设置起点
		void Set_start(int x, int y, int z);
		// 设置终点
		void Set_end(int x, int y, int z);
		// 设置障碍
		void Set_obstacle(int x, int y, int z);

		void Set_obstacle(vector<int> X, vector<int> Y, vector<int> Z);

		// 初始化地图数组
		void Init_array(vector<int> X, vector<int> Y, vector<int> Z);

		// 显示数组地图信息
		void View_array();
		// 显示A星地图信息
		void View_map();

		// 初始化节点的链表，返回一个初始化的链表！
		LinkList3 InitList();

		// 创建A星地图
		vector<vector<vector<LNode3>>> Creat_A_Map();
		void Creat_A_Map_void();
		// 边界检测
		bool isin(int x, int y, int z);

		// 查找A星地图起点的位置
		void find_start_LNode();
		// 获取A星地图起点的位置
		LNode3* Get_start_LNode();

		// 查找A星地图终点的位置!
		void find_end_LNode();
		// 获取A星地图终点的位置
		LNode3* Get_end_LNode();

		// 计算当前节点到上一个节点的G值
		int count_LNode_G(LNode3* curLNode, LNode3* aheadLNode);

		// 计算当前节点到终点的H值
		int count_LNode_H(LNode3* curLNode, LNode3* endLNode3);

		// 计算F
		int count_LNode_F(LNode3* curLNode);

		// 按F值由小到大插入节点到开放列表！
		void push_OpenList_Node(LinkList3 L, LNode3 *elem);

		// 取出开放列表中F值最小的节点
		LNode3* pop_OpenList_minNode(LinkList3 L_OpenList);

		// 头插法把用过的点插入到关闭列表中
		bool insert_Into_CloseList(LNode3* min_Open, LinkList3 L_CloseList);

		// 查看当前节点是否在开放列表中
		bool isExist_openList(LNode3* curLNode);

		// 查看当前节点是否在关闭列表中
		bool isExist_closeList(LNode3* curLNode);

		// 判断是否为障碍物
		bool isobstacle(LNode3* curLNode);

		// 判断是否应该加入到开放列表中
		bool isJoin(LNode3* cur);

		// 插入节点并更新相邻节点间的连接关系
		void insert_open(LNode3* Node, LNode3* ahead, LNode3* endLNode, LinkList3 open_list);

		// 广度优先搜索
		void check_around_curNode(LNode3* cur, LNode3* endLNode, LinkList3 open_list);

	public:
		// 地图的高
		int _height = 0;
		// 地图数组
		vector<vector<vector<int>>> _array3;

		// A星地图
		vector<vector<vector<LNode3>>> _map3;
		// 起点z坐标
		int _start_z = 0;
		// 终点z坐标
		int _end_z = 0;

		// A星地图的起始节点
		LNode3* start_LNode3 = NULL;
		// A星地图的最终节点
		LNode3* end_LNode3 = NULL;
};



#endif
