#ifndef __ROUTE_H__
#define __ROUTE_H__
#include<iostream>
#include<memory.h>
#include<vector>
#include <stdio.h>
#include<stdlib.h>
#include <queue>
#include <map>
#include <set>
#include <stack> 
#include<string.h>

#define MAX_PASS_POINT   50
#define MAX_POINT    601
#define OO         9999999  
static int flag=0;
using namespace std;
typedef struct node{
        int Edgeval;       //边权值
        int num;             //边的编号
}Node;
typedef struct path{
        int val;
        vector<int> vec;
}Path;
/*节点*/
typedef struct again{
	int num;            //点的编号
	struct again * next;
	struct again * prev;
} agNode;
/*自定义链表*/
typedef struct list{
	agNode * head;
	agNode * tail;
}List;

struct Nod {
    int id;
    Nod* path_father;   //路径中父节点
    int already;                //path中V'中节点数目（默认source和destination不在V'中>）
    int path_cost;
    Nod(int i) : id(i), path_cost(0), already(0), path_father(NULL) {};
    Nod(int i, int c, Nod* p, int a) : id(i), path_cost(c), path_father(p), already(a) {};

    bool inPath(int target) {
        Nod* path_rear = this;
        while (path_rear) {
            if (target == path_rear->id)
                return true;
            path_rear = path_rear->path_father;
        }
        return false;
    }
};
//pq中优先级的比较
struct Compare {
    //返回 node1优先级低于node2优先级
    bool operator()(Nod*node1, Nod*node2) {
        //经过一个V'节点的平均耗费
        if (!node1->already || !node2->already) {
            if (!node2->already)
                return false;
            return true;
        }
        //平均耗费越大，优先级越低
        return node1->path_cost / node1->already > node2->path_cost / node2->already;
    }
};

class Graph {
public:
    //图信息
    map<int, map<int, int> > neighbors;//id：<邻居id:最小边长>集合
    map<int, int> edges;//边id：cost
    set<int> V;//V'
    int source_id=0, destination_id=0;

    //优先队列（待扩展点）
    priority_queue<Nod*, vector<Nod*>, Compare> pq;

    //所有node地址集合，便于最后释放
    set<int> node_address;

    //最短路径长度初值为极大
    int min_cost = 100000;
    Nod* min_path;


    //TODO:初始化err
    Graph() {};
    Graph(char* graph[5000], int edge_num, char *condition) { 
	cout<<source_id<<endl;
	initialize(graph, edge_num, condition); };
    //~Graph() {
    //    for (auto a : node_address) {
    //        cout << "is going to delete " << ((Nod*)a)->id << endl;
    //        delete (Nod*)a;
    //    }
    //};


    //将当前扩展节点node的 不在node->path中的neighbor节点add到pq中
    void addNeighborsToPq(Nod* node) {
        //node的每一条出边
          for (map<int, int>::iterator pair0 = neighbors[node->id].begin(); pair0 != neighbors[node->id].end(); ++pair0)
	  {
            int neighbor_id = pair0->first, cost = pair0->second;//邻居节点、相应cost
            //已在path中，或未遍历完V情况下到了终点
            if (node->inPath(neighbor_id) || (node->already < V.size() && neighbor_id == destination_id))
                continue;

            Nod* neighbor_node = new Nod(neighbor_id,
                node->path_cost + cost,
                node,	//path上的父亲节点
                V.count(neighbor_id) ? node->already + 1 : node->already);
//            node_address.insert((int)&(*neighbor_node));
            //已经过V中全部点 且 该点是终点
            if (neighbor_node->already == V.size() && neighbor_id == destination_id)	 {
                if (neighbor_node->path_cost < min_cost) {
             //       cout << "change min_cost from " << min_cost;
                    min_cost = neighbor_node->path_cost, min_path = neighbor_node;
             //       cout << " to " << min_cost << endl;
             //       printPath(min_path);
                  flag++;
                }
            }
            else
                pq.push(neighbor_node);
        }
    }

    //取pq的top并将其pop
    Nod* priorityPop() {
        Nod* result = pq.top();
        pq.pop();
        return result;
    }

    void printPath(Nod* path_rear) {
        cout << "path : ";
        stack<int> s;
        while (path_rear) {
            s.push(path_rear->id);
            path_rear = path_rear->path_father;
        }
        while (!s.empty())
            cout << s.top() << " ", s.pop();
        cout << endl;
    }



    Nod *search1() {
        Nod* source_node = new Nod(source_id);
        pq.push(source_node);
        while (!pq.empty()) {
            Nod* node = priorityPop();
           // if (node->path_cost >= min_cost)
           //     continue;

            if(node->path_cost + (V.size() - node->already + 1) *4>= min_cost)
     	       continue;
           

            addNeighborsToPq(node);
            if(flag>=3)
			break;
        }
	return min_path;
    }
    Nod *search2() {
        Nod* source_node = new Nod(source_id);
        pq.push(source_node);
        while (!pq.empty()) {
            Nod* node = priorityPop();
            //if (node->path_cost >= min_cost)
            //    continue;
            if(node->path_cost + (V.size() - node->already + 1) *4>= min_cost)
               continue;


	    addNeighborsToPq(node);
            if(flag>=2)
                        break;
        }
        return min_path;
    }

    Nod *search3() {
        Nod* source_node = new Nod(source_id);
        pq.push(source_node);
        while (!pq.empty()) {
            Nod* node = priorityPop();
           // if (node->path_cost >= min_cost)
           //     continue;
            if(node->path_cost + (V.size() - node->already + 1) *4>= min_cost)
               continue;

            addNeighborsToPq(node);
            if(flag>=1)
                        break;
        }
        return min_path;
    }




    void initialize(char *graph[5000], int edge_num, char *condition) {

	cout<<source_id<<endl;    
        //0.将topo.csv中的每一行数据提取出来
        int  **csvInfo = new int*[edge_num];
        for (int i = 0; i <edge_num; i++)
            csvInfo[i] = new int[4];


        for (int i = 0; i<edge_num; i++)
        {
            int col = 0;
            char *tok = strtok(graph[i], ",");
            while (tok != NULL)
            {
                csvInfo[i][col++] = atoi(tok);
                tok = strtok(NULL, ",");
            }
        }


        for (int i = 0; i < edge_num; i++) {

            //LinkID

            int  l_id = csvInfo[i][0];

            //SourceID
            int  s_id = csvInfo[i][1];
            //DestinationID
            int  d_id = csvInfo[i][2];
            //Cost
            int  c = csvInfo[i][3];
            //设置图信息
            edges[l_id] = c;
            neighbors[s_id][d_id] = neighbors[s_id][d_id] == 0 ? c : min(neighbors[s_id][d_id], c);

        }
        int index = 0;
        while (condition[index] != ',')
            source_id = source_id * 10 + condition[index++] - '0';
//	cout<<"source_id"<<source_id<<endl<<endl;
        index++;
        while (condition[index] != ',')
            destination_id = destination_id * 10 + condition[index++] - '0';
        index++;


        while (condition[index]) {
            int node_id = 0;
            while (condition[index] && condition[index] != '|')
                node_id = node_id * 10 + condition[index++] - '0';
            if (condition[index])
                index++;
            V.insert(node_id);
        }
    }

    void printGraph() {
        cout << "printGraph !!!" << endl;

        cout << "V' : ";
        for (set<int>::iterator node = V.begin();node!=V.end();++node)
            cout << *node << " | ";
        cout << endl;

        cout << "source_id : " << source_id << endl;
        cout << "destination_id : " << destination_id << endl;

        cout << "edges:" << endl;
        for (map<int,int>::iterator edge = edges.begin();edge!=edges.end();++edge)
            cout << "    LinkID " << edge->first << " : cost " << edge->second << endl;
        cout << "neighbors:" << endl;
	  for (map<int, map<int, int> >::iterator it = neighbors.begin(); it != neighbors.end(); ++it)
            for (map<int, int>::iterator neighbor = it->second.begin(); neighbor != it->second.end(); ++neighbor)
                cout << "    source_id " << it->first << " : destination_id " << neighbor->first << " : shortest_edges_len " << neighbor->second << endl;
        


        cout << endl;
    }

};


/*创建一个节点*/
agNode * create_node(int num);

/*链表初始化*/
void list_init(List * list);

/*追加一个节点*/
void list_append(List * list,int num);

/*链表长度*/
int list_size(List * list);

/*打印链表*/
void list_print(List * list);

void  search_route(char *graph[5000], int edge_num, char *condition);
/*空间分配和释放*/
int** init(int m,int n);
void deinit(int ** p,int m,int n);
/*点数统计*/
int pointCount(int ** csvInfo,int Edgenum);

/*构建邻接矩阵*/
void createAdjmax(Node ** adjMat,int** csvInfo,int Edgenum,int Pointnum);


/*最短路Dijkstra算法*/
void DijkstraPath(Node ** matrix,int n,int *dist,int *path,int v0) ;

/*打印最短路径上的各个顶点 */
vector<int> showPath(int *path,int v,int v0);

/*填写最短路径表*/
void routineChart(Node ** matrix,Path ** homebestPath,int * passPoint,int * dist,int * path,int passNum,int n);

/*起点到中间点，中间点到终点的最短距离统计*/
void start_pass_end_distance(Node ** matrix,Path ** pa,int * dist,int * path,int * centrePoint,int start,int end,int n,int passNum);

/*排列优化*/
//void perm(Path ** pa,int start,int end,int passNum,int * road ,vector<int>& vec,int * passPoint,int *flag,int pointNum ,int k, int m) ;
void perm(Path ** pa,int start,int end,int passNum,int * road ,int * have,int * passPoint, int pointNum,int k, int m);


/*点重复的处理*/
bool againLoad(Node ** matrix,Path ** path,int * have,bool * flag,vector<int>& loadPoint,int start,int end,int pointNum,int passNum);
/*排列优化*/
void perm(Path ** pa,int start,int end,int passNum,int * road ,vector<int>& vec,int * passPoint,int *flag,int pointNum ,int k, int m) ;
#endif


