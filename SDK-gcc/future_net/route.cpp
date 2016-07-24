#include "route.h"
#include "lib_record.h"
#include <stdio.h>
#include<stdlib.h>
#include<stack>
#include<algorithm>
#include<iostream>
//你要完成的功能总入口
void search_route(char *topo[5000], int edge_num, char *demand)
{
       	//0.将topo.csv中的每一行数据提取出来
	int  **csvInfo=init(edge_num,4);
	
	for (int i=0;i<edge_num;i++)
    	{   
        	int col=0;
       		char *tok = strtok(topo[i],",");
           	while (tok!=NULL)
            	{
            		csvInfo[i][col++] = atoi(tok);
           		tok = strtok(NULL, ",");
            	}
   	}
      	//1.创建临接矩阵
       	int pointNum=0;
       	pointNum = pointCount(csvInfo,edge_num);//获取点的数量
       	Node ** adjMat = new Node*[pointNum];   //adjMat邻接矩阵
	for (int i = 0;i < pointNum;i++)
		adjMat[i] = new Node[pointNum];
        createAdjmax(adjMat,csvInfo,edge_num,pointNum);
       	//2.获取指定路径起点，终点和必须经过的点
      	int start, end;              		//路径的起点和终点
     	int *centrePoint=new int[50];   	//指定的路径
      	char *tokenPtr = strtok(demand, ",");
       	start = atoi(tokenPtr);  		 
       	tokenPtr = strtok(NULL, ",");
      	end = atoi(tokenPtr);;    		

      	tokenPtr = strtok(NULL, ",");
      	char *DestToken =strtok(tokenPtr,"|");
     	int d=0;
    	while (DestToken!=NULL)
     	{
        	centrePoint[d++] = atoi(DestToken);
       		DestToken = strtok(NULL, "|");
     	}
     	int passPoint=d;   

	//3.计算

     	int * dist = new int[pointNum];
	int * path = new int[pointNum];
       
	Path ** PA = new Path*[pointNum];
	for (int i = 0;i < pointNum;i++)
		PA[i] = new Path[pointNum];
        
	for (int i = 0;i < pointNum;i++)
		for (int j = 0;j < pointNum;j++)
			if (i == j)
				PA[i][j].val = 0;
  			else
				PA[i][j].val = OO;

	
  	routineChart(adjMat,PA,centrePoint,dist,path,passPoint,pointNum);
     

     	start_pass_end_distance(adjMat,PA,dist,path,centrePoint,start,end,pointNum,passPoint);

	int road = OO;
      //  int * flag = new int[pointNum];
	bool * flag = new bool[pointNum];         //判断已经使用过的点的标志
	memset(flag,false,sizeof(bool) * pointNum); //初始化全部没有使用
	int * have = new int[pointNum];        //必经点的最佳排列方式
	vector<int> loadPoint;     //最终的路径顶点号

 	int Min = 0,temp,sum = OO;
	List list;
	list_init(&list);
	int answer=0;

	//计算过程分析
//	if (passPoint <11&&passPoint > 0){
//	}      
	if(passPoint<=5){
        	int * dist = new int[pointNum];
        	int * path = new int[pointNum];
      		Path ** PA = new Path*[pointNum];
        	for (int i = 0;i < pointNum;i++)
                	PA[i] = new Path[pointNum];
        	for (int i = 0;i < pointNum;i++){
                	for (int j = 0;j < pointNum;j++)
			{
                        	if (i == j)
                                	PA[i][j].val = 0;
                        	else
                                	PA[i][j].val = OO;
                	}
        	}
        	routineChart(adjMat,PA,centrePoint,dist,path,passPoint,pointNum);
       		start_pass_end_distance(adjMat,PA,dist,path,centrePoint,start,end,pointNum,passPoint);
        	int road = OO;
        	int * flag = new int[pointNum];
        	vector<int> bestpath;
        	perm(PA,start,end,passPoint,&road,bestpath,centrePoint,flag,pointNum,0, passPoint-1);
        	if(road>=OO)
                	  return;
        	unsigned short result[150]={0};
       		for (size_t i = 0;i < bestpath.size()-1;i++){
                	 result[i]=adjMat[bestpath[i]][bestpath[i+1]].num;
        	}
      		//将最短路径保存到文件中
       		for (size_t i = 0; i < bestpath.size()-1; i++)
                	        record_result(result[i]);

      }
/*
	//处理passPoint结点大于6的情况
      else if((edge_num>=1200&&edge_num<=2000||edge_num<1000)&&passPoint<=22){
        	Graph g;
		for(int i=0;i<edge_num;i++)
		{
	  		//LinkID
          		int  l_id = csvInfo[i][0];
    		        //SourceID
          		int  s_id = csvInfo[i][1];
         		//DestinationID
          		int  d_id = csvInfo[i][2];
          		//Cost
          		int  c = csvInfo[i][3];
          		//设置图信息
          		g.edges[l_id] = c;
                        g.neighbors[s_id][d_id] = g.neighbors[s_id][d_id] == 0 ? c : min(g.neighbors[s_id][d_id], c);
		}
		//起点和终点
		g.source_id=start;
		g.destination_id=end;
        	//保存路径信息
		for(int l=0;l<passPoint;l++)
             	g.V.insert(centrePoint[l]);
		//路径搜索
		Nod *path_rear=NULL;
		if(edge_num<600&&passPoint<=7)
      			  path_rear= g.search1();
		else if(edge_num<=900&&passPoint<=9)
 			  path_rear= g.search2();
		else
		          path_rear=g.search3();
		
		//将路径保存到栈中  
        	stack<int> s;
        	while (path_rear) {
	    		s.push(path_rear->id);
            		path_rear = path_rear->path_father;
        	}
        	int i=0,j=0,k=0;
       	 	unsigned short result[150]={0};
		//在矩阵中查询路径对应的索引 
        	while (s.size()>1)
        	{
                	i=s.top();s.pop();
                	j=s.top();            
             		result[k++]=adjMat[i][j].num;
        	}
		//将路径信息写入文件中
        	for (int q = 0; q < k; q++)
        	{	
                        record_result(result[q]);
        	}

	}

*/
	else{
		bool * flag10 = new  bool[pointNum];
		memset(flag10,false,sizeof(bool) * pointNum);
		agNode * find = NULL;
		agNode * node = NULL;
		PA[start][end].val = 0;
		list_append(&list,start);
		list_append(&list,end);
		while(list_size(&list) < passPoint+2){
			for (int i = 0;i < passPoint;i++){
				if (flag10[centrePoint[i]])
					continue;
				for (find = list.head;find->next;find=find->next){
					if (Min - PA[find->num][find->next->num].val + PA[find->num][centrePoint[i]].val + PA[centrePoint[i]][find->next->num].val < sum){
						sum = Min - PA[find->num][find->next->num].val + PA[find->num][centrePoint[i]].val + PA[centrePoint[i]][find->next->num].val;
						temp = centrePoint[i];
						node = find;
					}
				}
			}
			flag10[temp] = true;
			Min = sum;
			sum = OO;
			for (find = list.head;find;find=find->next){
				if (find == node){
						agNode * nn = create_node(temp);
						nn->next = find->next;
						find->next->prev = nn;
						find->next = nn;
						nn->prev = find;
				}
			}
		}
		int m = 0;
		for (find = list.head->next;find->next;find = find->next){
			have[m++] = find->num;
		}
	        do     {
		 	bool xx = againLoad(adjMat,PA,have,flag,loadPoint,start,end,pointNum,passPoint);
			if (xx)
				break;
			memset(flag,false,sizeof(bool) * pointNum); //初始化全部没有使用
			loadPoint.clear() ;//清空
	        	}while(next_permutation(have,have + passPoint));	
		for (int i = 0;i < loadPoint.size()-1;i++){
			answer += adjMat[loadPoint[i]][loadPoint[i+1]].Edgeval;
		}
        	if(answer>=OO)
 			  return;  

		cout<<answer<<endl;
        	unsigned short result[150]={0};
		for (size_t i = 0;i < loadPoint.size()-1;i++)
			 result[i]=adjMat[loadPoint[i]][loadPoint[i+1]].num;

       		//将最短路径保存到文件中
       		for (size_t i = 0; i < loadPoint.size()-1; i++)
                	 	record_result(result[i]);	
	
       }
    	/***********内存释放区***********************************/

/*	deinit(csvInfo,edge_num,4);      
	//deinit(passedge,passPoint,passPoint);
	delete[] dist;
	delete[] path;
	for (int i = 0;i < pointNum;i++){
		delete[] adjMat[i];
		delete[] PA[i];
	}
	delete[] adjMat;
	delete[] PA;
	delete[] flag;
	delete[] have;
*/
}

/*内存空间分配*/
int** init(int m,int n){
	int ** space = new int*[m];
	for (int i = 0;i < m;i++)
		space[i] = new int[n];
	return space;
}
/*内存空间释放函数*/
void deinit(int ** p,int m,int n){
	for (int i = 0; i < m;i++)
		delete[] p[i];
	delete[] p;
}
/*点数统计*/
int pointCount(int ** csvInfo,int Edgenum){
	int Pointnum = 0;
	int * point = new int[MAX_POINT];
	memset(point,-1,sizeof(int)*MAX_POINT);
	for (int i = 0;i < Edgenum;i++){
		point[csvInfo[i][1]]++;
		point[csvInfo[i][2]]++;
	}
	for (int i = 0;i < MAX_POINT;i++)
		if (point[i] != -1)
			Pointnum++;
	delete[] point;
	return Pointnum;
}
/*构建邻接矩阵*/
void createAdjmax(Node ** adjMat,int** csvInfo,int Edgenum,int Pointnum){
	for (int i = 0;i < Pointnum;i++)
		for (int j = 0;j < Pointnum;j++){
			adjMat[i][j].Edgeval = OO;
			adjMat[i][j].num = -1;
		}
	for (int i = 0;i < Edgenum;i++){
		adjMat[csvInfo[i][1]][csvInfo[i][2]].Edgeval = csvInfo[i][3];
		adjMat[csvInfo[i][1]][csvInfo[i][2]].num = csvInfo[i][0];
	}
}

/*填写最短路径表*/
void routineChart(Node ** matrix,Path ** homebestPath,int * passPoint,int * dist,int * path,int passNum,int n){
	for (int i = 0;i < passNum;i++){
		DijkstraPath(matrix,n,dist,path,passPoint[i]);

		for (int j = 0;j < passNum;j++)
			if (i != j){
					  if (dist[passPoint[j]] == OO){
						homebestPath[passPoint[i]][passPoint[j]].val = OO;
						  }
				 	  else{
				homebestPath[passPoint[i]][passPoint[j]].val = dist[passPoint[j]];
				homebestPath[passPoint[i]][passPoint[j]].vec = showPath(path,passPoint[j],passPoint[i]);
			   	           }
			}
}
}

/*打印最短路径上的各个顶点 */
vector<int> showPath(int *path,int v,int v0)   //打印最短路径上的各个顶点 
{
    stack<int> s;
	vector<int> vec;
    while(v!=v0)
    {
        s.push(v);
        v=path[v];
    }
    s.push(v);
   while(!s.empty())
    {
		vec.push_back(s.top());
        s.pop();
    }
   return vec;
}
/*最短路Dijkstra算法*/
void DijkstraPath(Node ** matrix,int n,int *dist,int *path,int v0)  
{
    int i,j,k;
    bool *visited= new bool[n];          
    for(i=0;i<n;i++)     //初始化 
    {
		if(matrix[v0][i].Edgeval!=OO&&i!=v0)                                   
        { 
			dist[i]=matrix[v0][i].Edgeval;                                           
            path[i]=v0;                                                            
        }
        else                                                                             
        {
            dist[i]=OO;    
            path[i]=-1;                 
        }
        visited[i]=false;             //初始状态表示未使用
        path[v0]=v0;                 //从v0到v0的前一个顶点是v0
        dist[v0]=0;                    //从v0到v0的距离是0
    }
    visited[v0]=true;                //v0点已经被使用过了
    int u;
    for(i=1;i<n;i++)     //循环扩展n-1次 
    {
        int min = OO;
      
        for(j=0;j<n;j++)    //寻找未被扩展的权值最小的顶点 
        {
            if(visited[j] == false && dist[j] < min)      //找出最小的dist值
            {
                min = dist[j];
                u = j;                                        //记录下此时的最小点   
            }
        } 
        visited[u] = true;                //u点被使用
        for(k = 0;k < n;k++)   //更新dist数组的值和路径的值 
        {
			if(visited[k]==false&&matrix[u][k].Edgeval!=OO&&min+matrix[u][k].Edgeval<dist[k])
            {
				dist[k]=min+matrix[u][k].Edgeval;
                path[k]=u; 
            }
        }        
    }  
	delete[] visited;
}/*起点到中间点，中间点到终点的最短距离统计*/
void start_pass_end_distance(Node ** matrix,Path ** pa,int * dist,int * path,int * centrePoint,int start,int end,int n,int passNum){
	DijkstraPath(matrix,n,dist,path,start);
	for (int i = 0;i < passNum;i++){
		pa[start][centrePoint[i]].val = dist[centrePoint[i]];
		pa[start][centrePoint[i]].vec = showPath(path,centrePoint[i],start);
	}
	/*中间点到终点*/
	for (int i = 0;i < passNum;i++){
		DijkstraPath(matrix,n,dist,path,centrePoint[i]);
		pa[centrePoint[i]][end].val = dist[end];
		pa[centrePoint[i]][end].vec = showPath(path,end,centrePoint[i]);
	}
}  
/*排列优化*/
void perm(Path ** pa,int start,int end,int passNum,int * road ,int * have,int * passPoint, int pointNum,int k, int m) 
{     
    if(k >= m)     
    {    
		int temp = -1,i;
		temp = pa[start][passPoint[0]].val;

		for (int x = 0;x < passNum-1;x++)
			temp += pa[passPoint[x]][passPoint[x+1]].val;
		
		temp += pa[passPoint[passNum-1]][end].val;
		if (temp < *road){
			*road = temp;
			for (int j = 0;j < passNum;j++)
				have[j] = passPoint[j];
		}
    }     
    else     
    {         
        for(int i = k; i <= m; i++)         
        {             
            swap(passPoint[k], passPoint[i]);             
            perm(pa,start,end,passNum,road, have,passPoint,pointNum,k + 1, m);             
            swap(passPoint[k], passPoint[i]);             
        }     
    } 
}  
/*排列优化*/
void perm(Path ** pa,int start,int end,int passNum,int * road ,vector<int>& vec,int * passPoint,int *flag,int pointNum, int k, int m)
{
    if(k >= m)
    {
                int temp = -1;
                temp = pa[start][passPoint[0]].val;
                vector<int> edge1 = pa[start][passPoint[0]].vec;
                for (int x = 0;x < passNum-1;x++){
                        temp += pa[passPoint[x]][passPoint[x+1]].val;
                        vector<int> edge2 = pa[passPoint[x]][passPoint[x+1]].vec;
                        for (size_t m = 0;m < edge2.size()-1;m++)
                                edge1.push_back(edge2[m+1]);
                }
                temp += pa[passPoint[passNum-1]][end].val;
                vector<int> edge3 = pa[passPoint[passNum-1]][end].vec;
                for (size_t m = 0;m < edge3.size()-1;m++)
                        edge1.push_back(edge3[m+1]);

                memset(flag, 0, sizeof(int)*pointNum);
                for (size_t m = 0; m < edge1.size(); m++){

                        flag[edge1[m]]++;
                        if (flag[edge1[m]] >= 2)
                          return;
                }
        if (temp < *road){
                        *road = temp;
                        vec = edge1;
                }
    }
    else
    {
        for(int i = k; i <= m; i++)
        {
            swap(passPoint[k], passPoint[i]);
            perm(pa,start,end,passNum,road, vec,passPoint,flag,pointNum,k + 1, m);     
            swap(passPoint[k], passPoint[i]);
        }
    }
}
 

/*创建一个节点*/
agNode * create_node(int num){
	agNode * node = (agNode*)malloc(sizeof(agNode));
	node->num = num;
	node->prev = NULL;
	node->next = NULL;
	return node;
}
/*链表初始化*/
void list_init(List * list){
	list->head = NULL;
	list->tail = NULL;
}

/*追加一个节点*/
void list_append(List * list,int num){
	agNode * node = create_node(num);
	if (!list->head && !list->tail){
		list->head = list->tail = node;
		node->next = NULL;
		node->prev = NULL;
	}
	else{
		list->tail->next = node;
		node->prev = list->tail;
		list->tail = node;
	}
}
/*链表长度*/
int list_size(List * list){
	agNode * find = NULL;
	int size = 0;
	for (find = list->head;find;find = find->next)
		size++;
	return size;
}
/*打印链表*/
void list_print(List * list){
	agNode * find = NULL;
	for (find = list->head;find;find = find->next)
      		cout <<find->num <<"-";
	cout << endl;
}
/*重复点的路径重新确定*/
/*
static vector<int> Load1(Node ** matrix,bool * flag,int v0,int end,int n){
    int i,j,k;
	int * dist = new int[n];
	int * path = new int[n];
    bool *visited= new bool[n];          
    for(i=0;i<n;i++)     //初始化 
    {
		if(matrix[v0][i].Edgeval!=OO&&i!=v0&&flag[i] == false)                                   
        { 
			dist[i]=matrix[v0][i].Edgeval;                                           
            path[i]=v0;                                                            
        }
        else                                                                             
        {
            dist[i]=OO;    
            path[i]=-1;                 
        }
        visited[i]=false;             //初始状态表示未使用
        path[v0]=v0;                 //从v0到v0的前一个顶点是v0
        dist[v0]=0;                    //从v0到v0的距离是0
    }
    visited[v0]=true;                //v0点已经被使用过了
	for (int m = 0;m < n;m++)
		if (flag[m])
			visited[m] = true;
	int u;
    for(i=1;i<n;i++)     //循环扩展n-1次 
    {
        int min = OO;
		/////////////////////////////////////////////////////////////////////
		if (visited[end]){
			vector<int> temp = showPath(path,end,v0);
			return temp;
		}
		////////////////////////////////////////////////////////////////////
        for(j=0;j<n;j++)    //寻找未被扩展的权值最小的顶点 
        {
            if(visited[j] == false && dist[j] < min)      //找出最小的dist值
            {
                min = dist[j];
                u = j;                                        //记录下此时的最小点   
            }
        } 
        visited[u] = true;                //u点被使用
        for(k = 0;k < n;k++)   //更新dist数组的值和路径的值 
        {
			if(visited[k]==false&&matrix[u][k].Edgeval!=OO&&min+matrix[u][k].Edgeval<dist[k])
            {
				dist[k]=min+matrix[u][k].Edgeval;
                path[k]=u; 
            }
        }        
    }  
	vector<int> temp = showPath(path,end,v0);
	delete[] visited;
	delete[] path;
	delete[] dist;
	return temp;
}
*/
/*重复点的路径重新确定*/
static vector<int> Load1(Node ** matrix,bool * flag,int v0,int end,int n){
    int i,j,k;
	vector<int> temp;
	int * dist = new int[n];
	int * path = new int[n];
    bool *visited= new bool[n];          
    for(i=0;i<n;i++)     //初始化 
    {
		if(matrix[v0][i].Edgeval!=OO&&i!=v0&&flag[i] == false)                                   
        { 
			dist[i]=matrix[v0][i].Edgeval;                                           
            path[i]=v0;                                                            
        }
        else                                                                             
        {
            dist[i]=OO;    
            path[i]=-1;                 
        }
        visited[i]=false;             //初始状态表示未使用
        path[v0]=v0;                 //从v0到v0的前一个顶点是v0
        dist[v0]=0;                    //从v0到v0的距离是0
    }
    visited[v0]=true;                //v0点已经被使用过了
	for (int m = 0;m < n;m++)
		if (flag[m])
			visited[m] = true;
	int u = -100;
    for(i=1;i<n;i++)     //循环扩展n-1次 
    {
        int min = OO;
		/////////////////////////////////////////////////////////////////////
		/*if (visited[end]){
			vector<int> temp = showPath(path,end,v0);
			return temp;
		}*/
		////////////////////////////////////////////////////////////////////
        for(j=0;j<n;j++)    //寻找未被扩展的权值最小的顶点 
        {
            if(visited[j] == false && dist[j] < min)      //找出最小的dist值
            {
                min = dist[j];
                u = j;                                        //记录下此时的最小点   
            }
        } 		if (u == -100)
			return temp;
        visited[u] = true;                //u点被使用
        for(k = 0;k < n;k++)   //更新dist数组的值和路径的值 
        {
			if(visited[k]==false&&matrix[u][k].Edgeval!=OO&&min+matrix[u][k].Edgeval<dist[k])
            {
				dist[k]=min+matrix[u][k].Edgeval;
                path[k]=u; 
            }
        }        
    }  
	if (dist[end] == OO)
		return temp;
	else{
		vector<int> temp = showPath(path,end,v0);
		return temp;
	}
	//cout << dist[v0]<<"----------"<<dist[end]<< endl;
	
	delete[] visited;
	delete[] path;
	delete[] dist;
	return temp;
}


/*分支限界*/
/*
void againLoad(Node ** matrix,Path ** path,int * have,bool * flag,vector<int>& loadPoint,int start,int end,int pointNum,int passNum){
//	for (int i = 0;i < passNum;i++)
//		cout << have[i] <<" ";
//	cout << endl;
	for (int i = 0;i < path[start][have[0]].vec.size();i++){
		loadPoint.push_back(path[start][have[0]].vec[i]);      //路径点依次加入到集合中
		flag[path[start][have[0]].vec[i]] = true;                      //已经使用过了
	}
	for (int i = 0;i < passNum-1;i++){
		int kk = i+1;
		while(flag[have[kk]]){ 
			kk++;
		}
		if (kk>i+1){
			/////////////////////////////////////
			昨天写到这个位置了*/
/*				vector<int> temp = Load1(matrix,flag,have[i],have[kk],pointNum);
				for (int k = 1;k < temp.size();k++){
					loadPoint.push_back(temp[k]);
					flag[temp[k]] = true;
				}
				if (kk == passNum-1)
					break;
				if(kk > passNum - 1)
					return;
				i = kk-1;
				continue;
			}
		if (path[have[i]][have[i+1]].vec.size() == 2){
			for (int x = 1;x < path[have[i]][have[i+1]].vec.size();x++){
						loadPoint.push_back(path[have[i]][have[i+1]].vec[x]);
						flag[path[have[i]][have[i+1]].vec[x]] = true;
					}
				}
		else{
			for (int j = 1;j < path[have[i]][have[i+1]].vec.size()-1;j++){
				if (flag[path[have[i]][have[i+1]].vec[j]]){
					vector<int> temp = Load1(matrix,flag,have[i],have[i+1],pointNum);
					for (int k = 1;k < temp.size();k++){
						loadPoint.push_back(temp[k]);
						flag[temp[k]] = true;
					}
					break;
				}
				if (j == path[have[i]][have[i+1]].vec.size()-2){
					for (int x = 1;x < path[have[i]][have[i+1]].vec.size();x++){
						loadPoint.push_back(path[have[i]][have[i+1]].vec[x]);
						flag[path[have[i]][have[i+1]].vec[x]] = true;
					}
				}
			}
		}	
	}
	for (int i = 1;i < path[have[passNum-1]][end].vec.size();i++){
		if (flag[path[have[passNum-1]][end].vec[i]]){
			vector<int> temp = Load1(matrix,flag,have[passNum-1],end,pointNum);
			for (int j = 1;j < temp.size();j++)
				loadPoint.push_back(temp[j]);
			return;
		}
	}
			for (int j = 1;j < path[have[passNum-1]][end].vec.size();j++)
				loadPoint.push_back(path[have[passNum-1]][end].vec[j]);
}
*/

/*分支限界*/
bool againLoad(Node ** matrix,Path ** path,int * have,bool * flag,vector<int>& loadPoint,int start,int end,int pointNum,int passNum){
	flag[end] = true;
	if (path[start][have[0]].vec.size() == 0)
		return false;
	for (int i = 0;i < path[start][have[0]].vec.size();i++){  
		loadPoint.push_back(path[start][have[0]].vec[i]);     
		flag[path[start][have[0]].vec[i]] = true;                      
	}
	/******************************************************************************************/
	for (int i = 0;i < passNum;i++){
		int pos = i;
		if (i == passNum-1)
			goto loop;
  		while(flag[have[++pos]]){
			if (pos == passNum-1){//说明必经过的点的最后一个被使用过了
				vector<int> temp = Load1(matrix,flag,have[i],have[end],pointNum);
				if (temp.size() == 0)
					return false;
				for (int k = 1;k < temp.size();k++){
					loadPoint.push_back(temp[k]);
					flag[temp[k]] = true;
				}
				return true;
			}
		}   
		/******************************************************************************************/
		if (path[have[i]][have[pos]].vec.size() == 0)
			return false;
		if (path[have[i]][have[pos]].vec.size() == 2){
			loadPoint.push_back(have[pos]);
			flag[have[pos]] = true;
			i = pos - 1;
			continue;
		}
		/*****************************************************************************************/
		for (int j = 1;j < path[have[i]][have[pos]].vec.size()-1;j++){
			if (flag[path[have[i]][have[pos]].vec[j]]){
				vector<int> temp = Load1(matrix,flag,have[i],have[pos],pointNum);
				if (temp.size() == 0)
					return false;
				for (int k = 1;k < temp.size();k++){
					loadPoint.push_back(temp[k]);
					flag[temp[k]] = true;
				}
				i = pos - 1;
				break;
			}
			if (j == path[have[i]][have[pos]].vec.size()-2){
				for (int k = 1;k < path[have[i]][have[pos]].vec.size();k++){
					loadPoint.push_back(path[have[i]][have[pos]].vec[k]);
					flag[path[have[i]][have[pos]].vec[k]] = true;
				}
				i = pos-1;
			}
		}
	}
	/*******************************************************************************************/
loop:flag[end] = false;
	if (path[have[passNum-1]][end].vec.size() == 0)
		return false;
	if (path[have[passNum-1]][end].vec.size() == 2){
		loadPoint.push_back(end);
		return true;
	}
	for (int j = 1;j < path[have[passNum - 1]][end].vec.size() - 1;j++){
		if (flag[path[have[passNum-1]][end].vec[j]]){
			vector<int> temp = Load1(matrix,flag,have[passNum-1],end,pointNum);
				if (temp.size() == 0)
					return false;
				for (int k = 1;k < temp.size();k++){
					loadPoint.push_back(temp[k]);
					flag[temp[k]] = true;
				}
				return true;
		}
		if (j == path[have[passNum-1]][end].vec.size()-2){
				for (int k = 1;k < path[have[passNum-1]][end].vec.size();k++){
					loadPoint.push_back(path[have[passNum-1]][end].vec[k]);
				}
				return true;
			}
	}
	return true;
	
}
void printPath(Nod* path_rear) {
        cout << "path : ";
        stack<int> s;
        while (path_rear) {
            s.push(path_rear->id);
            path_rear = path_rear->path_father;
        }

        while (!s.empty())
        {
            cout << s.top() << " ", s.pop();
        }
         cout << endl;
}

