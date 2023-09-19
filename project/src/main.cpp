// C++ Standard Libraries
#include <iostream>
// Third Party
#if defined(LINUX) || defined(MINGW)
    #include <SDL2/SDL.h>
#else // This works for Mac
    #include <SDL.h>
#endif

#include <fstream>
#include <vector>
#include <map>
#include <cassert>
#include <numeric> // for iota
#include <algorithm>
#include <set>
#include <queue>
#include <cmath>
#include <unordered_set>
#include <unordered_map>

#include "Math.hpp"

// A node structure for each node in the A* graph search.
// Calling this a 'node' is accurate, but we might more generally
// think of this as a 'waypoint' in an API like an a game for instance.
// For simplicity, we can assume our node otherwise is positioned in
// a grid for this assignment otherwise.
struct Node{
    // Position grid
    int x;
    int y;
    // Size of the node in pixels
	size_t mCellSize{16};
    // neighbors which are adjacent to the current node
    std::set<Node*> mAdjacentNeighbors;
    // Properties of node
    // Is the node considered an obstacle
    bool mIsObstacle{false}; 
    int  mValue{0};

    // Specific to A* we need to keep track of the localScore, globalScore, and our parent
    // What is the 'parent' of the node (i.e. 'camefrom')
    Node* mParent{nullptr};
    // globalScore
    float globalScore = 99999999;
    float localScore  = 99999999;

    // Construct a new node
    Node(int xIndex, int yIndex){
        x = xIndex;
        y = yIndex; 
    }

    // Store a pointer to the neighbors of this node
    void AddNeighbor(Node* neighbor){
        // If we are an obstacle, cannot add neighbors
        if(mIsObstacle==true){
            return;
        }

        // Cannot add a neighbor as an obstacle
        if(neighbor->mIsObstacle==false){
            mAdjacentNeighbors.insert(neighbor);
        }
    }

    // Draw a single 'Node'
    void Draw(SDL_Renderer* renderer) const{
        // Create an SDL Rectangle
        SDL_Rect r;
        r.x = x*mCellSize;
        r.y = y*mCellSize;
        r.w = mCellSize;
        r.h = mCellSize;	

        // Draw our node
        if(0==mValue){
            SDL_SetRenderDrawColor(renderer,255,255,255,SDL_ALPHA_OPAQUE);
            SDL_RenderDrawRect(renderer,&r);
        }
        // Draw different states of the node 
        if(1==mValue){ // obstacle
            SDL_SetRenderDrawColor(renderer,160,32,32,SDL_ALPHA_OPAQUE);	
            SDL_RenderFillRect(renderer,&r);
        }
        if(2==mValue){ // start
            SDL_SetRenderDrawColor(renderer,32,255,32,SDL_ALPHA_OPAQUE);	
            SDL_RenderFillRect(renderer,&r);
        }
        if(3==mValue){ // goal
            SDL_SetRenderDrawColor(renderer,0,0,255,SDL_ALPHA_OPAQUE);	
            SDL_RenderFillRect(renderer,&r);
        }
    }

    // Render all of the neighbors
    void HighlightNeighbors(SDL_Renderer* renderer) const{
        for(auto& elements: mAdjacentNeighbors){
            SDL_Rect r;
            r.x = elements->x*elements->mCellSize;
            r.y = elements->y*elements->mCellSize;
            r.w = elements->mCellSize;
            r.h = elements->mCellSize;	
            SDL_SetRenderDrawColor(renderer,192,0,192,SDL_ALPHA_OPAQUE);	
            SDL_RenderDrawRect(renderer,&r);
        }
    }
};

// Comparator for the priority queue
struct CompareNode {
    bool operator()(Node* n1, Node* n2) {
        // return "true" if "n1" has a higher global score than "n2"
        return n1->globalScore > n2->globalScore;
    }
};

// Define the hash function for the Node.
struct NodeHash {
    std::size_t operator()(const Node* n) const {
        return std::hash<int>()(n->x) ^ std::hash<int>()(n->y);
    }
};

// Define the equality comparison for the Node.
struct NodeEqual {
    bool operator()(const Node* n1, const Node* n2) const {
        return n1->x == n2->x && n1->y == n2->y;
    }
};

// Overwriting how to compare two nodes
bool operator==(const Node &n1, const Node &n2) {
    return n1.x == n2.x && n1.y == n2.y;
}


//Needed to make my own PQ as C++ does not allow updates
class PriorityQueue {
private:
    std::multimap<float, Node*> map;
    std::unordered_map<Node*, std::multimap<float, Node*>::iterator> node_map;
public:
    bool empty() const { return map.empty(); }

    void push(Node* node) {
        node_map[node] = map.insert({node->globalScore, node});
    }

    void decrease_key(Node* node) {
        map.erase(node_map[node]);
        push(node);
    }

    Node* top() {
        return begin(map)->second;
    }

    void pop() {
        node_map.erase(begin(map)->second);
        map.erase(begin(map));
    }
};


// Grid for drawing the graph
struct Grid{
	size_t mXDimensions{40};
	size_t mYDimensions{30};
    //size_t mXDimensions{100};
	//size_t mYDimensions{90};
	size_t mCellSize{16};

	// A vector of all of the nodes in our graph 
    std::vector<Node> mConnectivityMatrix;

	// Constructor for the grid
    // This consists of setting up several rows and columns for each of our nodes
    // We will also build in any obstacles for the purpose of this here
	Grid(int x, int y) : mXDimensions(x), mYDimensions(y){
			// Allocate memory for the grid
			// Set every cell to 0 by default
            // Observe that I construct the 'row' first here
			for(int j=0; j < mYDimensions; j++){
			    for(int i=0; i < mXDimensions; i++){
					mConnectivityMatrix.emplace_back(i,j);
				}
			}
			// Somewhat arbitrary way to set some cells to 'obstacles'
			for(int i=4; i < mXDimensions; i+=4){
				for(int j=4; j < mYDimensions; j+=4){
					SetCell(i,j,1);
				}
			}
            // Add references to each of the neighbors
			for(int y=0; y < mYDimensions; y++){
			    for(int x=0; x < mXDimensions; x++){

                    if(x-1 > -1)
    					GetNodeReference(x,y).AddNeighbor(&GetNodeReference(x-1,y));
                    if(x+1 <= mXDimensions-1)
    					GetNodeReference(x,y).AddNeighbor(&GetNodeReference(x+1,y));
                    if(y-1 > -1)
    					GetNodeReference(x,y).AddNeighbor(&GetNodeReference(x,y-1));
                    if(y+1 <= mYDimensions-1)
    					GetNodeReference(x,y).AddNeighbor(&GetNodeReference(x,y+1));
				}
			}
	}

    // Retrieve a mutable reference to a node
    Node& GetNodeReference(int x, int y){
        return mConnectivityMatrix[y*mXDimensions+x];
    }

    Node* GetNodePointer(int x, int y) {
        return &mConnectivityMatrix[y*mXDimensions+x];
    }

	// Set a cell value
	int GetCellValue(int x, int y) const{
		return mConnectivityMatrix[y*mXDimensions+x].mValue;
	}

	// Modify a cell's value
    // Note: We have a special state of '1' for obstacles
    // Note: Design can be improved with an 'enum class'
	void SetCell(int x, int y, int value){
        if(value==1){
    		mConnectivityMatrix[y*mXDimensions+x].mIsObstacle = true;    
        }
		mConnectivityMatrix[y*mXDimensions+x].mValue = value;
	}

    // This helper function merely highlights the neighboring cells based on the 
    // mouse position. It is useful to see that the adjacency data is being correctly
    // populated.
    void DrawNeighbors(SDL_Renderer* renderer,int mouseX, int mouseY){
        int xCell = mouseX / mCellSize;	
        int yCell = mouseY / mCellSize;	
        if (yCell >= 0 && yCell < mYDimensions){
            if (xCell >= 0 && xCell < mXDimensions){
                GetNodeReference(xCell,yCell).HighlightNeighbors(renderer);
            }
        }
    }

    /**
     * Re-initialize nodes for A* search
    */
    void reIntializeNodes() {
        for(Node& curr : mConnectivityMatrix) {
            curr.mParent = nullptr;
            curr.globalScore = 99999999;
            curr.localScore = 99999999;
        }
    }

    void DrawPath(SDL_Renderer* renderer,int mouseX, int mouseY){
        int xCell = mouseX / mCellSize;	
        int yCell = mouseY / mCellSize;

        Node* curr = GetNodePointer(xCell,yCell);

        while (curr != NULL)
        {
            SDL_Rect r;
            r.x = curr->x*curr->mCellSize;
            r.y = curr->y*curr->mCellSize;
            r.w = curr->mCellSize;
            r.h = curr->mCellSize;
            SDL_SetRenderDrawColor(renderer,144,238,144,SDL_ALPHA_OPAQUE);	
            SDL_RenderFillRect(renderer,&r);
            curr = curr->mParent;
        }
    }

    void setObstacle(int mouseX, int mouseY) {
        // Determine the X and Y Cell that was clicked
        int xCell = mouseX / mCellSize;	
        int yCell = mouseY / mCellSize;

        SetCell(xCell, yCell, 1);

        Node* obstacle  = GetNodePointer(xCell,yCell);

        for (auto& element : obstacle->mAdjacentNeighbors) {
            element->mAdjacentNeighbors.erase(obstacle);        
        }

        obstacle->mAdjacentNeighbors.clear();
         	
    }


	// Set the goal for A* algorithm to run
	void SetGoal(int mouseX, int mouseY){
        // Determine the X and Y Cell that was clicked
        int xCell = mouseX / mCellSize;	
        int yCell = mouseY / mCellSize;	
        //std::cout << xCell << "," << yCell << std::endl;
        
        // Set the new goal state
        // Note: The goal cannot be an obstacle
        if(1 != GetCellValue(xCell,yCell)){
            // Starting position is always the 0th cell for this simulation
            SetCell(0,0,3);
            // Set the goal to a selected cell
            SetCell(xCell,yCell,2);
        }
        // Run A*
        RunAStar(0,0,xCell,yCell);
	}

    float Heuristic(Node* start, Node* end) {
        return std::sqrt(std::pow(start->x - end->x, 2) + std::pow(start->y - end->y, 2));
    }


	//A* path finding algorithm on the grid from 
	// a start to a finish
	void RunAStar(int startX, int startY, int goalX, int goalY){

       reIntializeNodes(); 

       Node* start  = GetNodePointer(startX,startY); 
       Node* end    = GetNodePointer(goalX,goalY);

       //std::cout << "Goal: " << end->x << " " << end->y << std::endl;

       //Make a priority queue which is the open list
       PriorityQueue openList;
       //Add start to open list
       start->localScore = 0.0f;
       start->globalScore = Heuristic(start, end) + start->localScore;
       openList.push(start);
       std::unordered_set<Node*, NodeHash, NodeEqual> openSet;
       openSet.insert(start); 

       //Make  a hasht set which is the closed list
       std::unordered_set<Node*, NodeHash, NodeEqual> closedList;

       Node* curr;
       //std::cout << "Starting" << std::endl; 
       while (!openList.empty())
       {
            curr = openList.top();
            openList.pop();
            //std::cout << curr->x << " " << curr->y << std::endl;
            closedList.insert(curr);

            //Terminating our search
            if (*curr == *end)
            {
                //std::cout << "Terminating" << std::endl;
                return;
            }

            for (auto& neighbor: curr->mAdjacentNeighbors) {
                if (closedList.find(neighbor) != closedList.end()) {
                    continue;
                }

                //std::cout << neighbor->x << " " << neighbor->y << std::endl;
                

                float tentativeLocalScore = curr->localScore + Heuristic(curr, neighbor);
                if (tentativeLocalScore < neighbor->localScore) {

                    neighbor->mParent = curr;
                    neighbor->localScore = tentativeLocalScore;
                    neighbor->globalScore = tentativeLocalScore + Heuristic(neighbor, end);   
                    
                    // If it's not in the open set, add it
                    if (openSet.find(neighbor) == openSet.end())
                    {
                        openSet.insert(neighbor);
                        openList.push(neighbor);
                    } else
                    {
                        openList.decrease_key(neighbor);
                    }
                    
                    
                    
                }
            }

       }
       
	}

	// Draw a square grid
    // This function is somewhat trivial because we can just loop through
    // the entire vector with each of our nodes. The nodes themselves store
    // the positions. 
    // Note: I still iterate through in a 'grid like' manner if the cells were
    //       to overlap in some meaningful way.
	void DrawAGrid(SDL_Renderer* renderer){
		for(int x=0; x < mXDimensions; x++){
			for(int y=0; y < mYDimensions; y++){
                mConnectivityMatrix[y*mXDimensions+x].Draw(renderer);
			}			
		}
	}
};


// Entry point to program
int main(int argc, char* argv[]){

    // Create our initial 'grid' of nodes
	Grid grid(36,30);


    // Create a window data type
    // This pointer will point to the 
    // window that is allocated from SDL_CreateWindow
    SDL_Window* window=nullptr;

    // Initialize the video subsystem.
    // iF it returns less than 1, then an
    // error code will be received.
    if(SDL_Init(SDL_INIT_VIDEO) < 0){
        std::cout << "SDL could not be initialized: " <<
                  SDL_GetError();
    }else{
        std::cout << "SDL video system is ready to go\n";
    }
    // Request a window to be created for our platform
    // The parameters are for the title, x and y position,
    // and the width and height of the window.
    window = SDL_CreateWindow("C++ SDL2 Window",20, 20, 640,480,SDL_WINDOW_SHOWN);

    SDL_Renderer* renderer = nullptr;
    renderer = SDL_CreateRenderer(window,-1,SDL_RENDERER_ACCELERATED);

    int goalX, goalY;
    int clickedOnce = 0;

    // Infinite loop for our application
    bool gameIsRunning = true;
    // Main application loop
    while(gameIsRunning){
        SDL_Event event;

        // (1) Handle Input
        // Start our event loop
        while(SDL_PollEvent(&event)){
            // Handle each specific event
            if(event.type == SDL_QUIT){
                gameIsRunning= false;
            }
        }
        // (2) Handle Updates
        
        // (3) Clear and Draw the Screen
        // Gives us a clear "canvas"
        SDL_SetRenderDrawColor(renderer,0,0,0,SDL_ALPHA_OPAQUE);
        SDL_RenderClear(renderer);


        // Draw the grid
		grid.DrawAGrid(renderer);

        // Set the goal point in the grid
        int mouseX,mouseY;
        
        Uint32 mouseState = SDL_GetMouseState(&mouseX,&mouseY);
        if(mouseState == SDL_BUTTON_LEFT){
            grid.SetGoal(mouseX,mouseY);
            clickedOnce = 1;
            goalX = mouseX;
            goalY = mouseY;            
        }

        //Added just to add more complexity
        if (mouseState & SDL_BUTTON(SDL_BUTTON_RIGHT))
        {
            //Add obstacle
            grid.setObstacle(mouseX, mouseY);

            if (goalX == mouseX && goalY == mouseY)
            {
                //Set goal is not able to change the neigbours for some reason
                grid.SetGoal(mouseX,mouseY);
                //This still works but only when grid is re-initialized at start
            }
            
        }
        


        if (clickedOnce == 1)
        {
            grid.DrawPath(renderer, goalX, goalY);
        }
        

        

        // Highlight neighbors
        grid.DrawNeighbors(renderer,mouseX,mouseY);

        // Finally show what we've drawn
        SDL_RenderPresent(renderer);

    }

    // We destroy our window. We are passing in the pointer
    // that points to the memory allocated by the 
    // 'SDL_CreateWindow' function. Remember, this is
    // a 'C-style' API, we don't have destructors.
    SDL_DestroyWindow(window);
    
    // our program.
    SDL_Quit();
    return 0;
}
