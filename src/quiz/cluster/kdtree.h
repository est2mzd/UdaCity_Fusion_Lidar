/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

    //--------------------------------------------------------------------------------------//
    // Lesson : Lidar-3-6 : Insert Points
    void insertHelper(Node** node, uint depth, std::vector<float> points, int id)
    {
        // If tree is empty.
        if(*node == nullptr)
        {
            // *node is Address of the Real Object
            // * is de-reference
            *node = new Node(points, id);
        }
        else
        {
            // Calculate current dim
            uint current_dim = depth % 2; // 0 or 1
            
            // if depth is even(0) -> compare X value
            // if depth is odd(1)  -> compare Y value
            if( points[current_dim] < ((*node)->point[current_dim])  )
            {
                insertHelper(&((*node)->left), depth+1, points, id);
            }
            else
            {
                insertHelper(&((*node)->right), depth+1, points, id);
            }
        }
        //std::cout << "id = " << id << std::endl;
    }

    //--------------------------------------------------------------------------------------//
    // Lesson : Lidar-3-6 : Insert Points
	void insert(std::vector<float> points, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
        insertHelper(&root, 0, points, id);
	}

    //--------------------------------------------------------------------------------------//
    // Lesson : Lidar-3-7 : Searching Points in a KD-Tree
    void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
    {
        if(node != nullptr)
        {
            // Check if node is in the box of target torelance
            bool flagXMin = node->point[0] >= (target[0]-distanceTol);
            bool flagXMax = node->point[0] <= (target[0]+distanceTol);
            bool flagYMin = node->point[1] >= (target[1]-distanceTol);
            bool flagYMax = node->point[1] <= (target[1]+distanceTol);
            //
            if( flagXMin && flagXMax && flagYMin && flagYMax )
            {
                float dx = node->point[0] - target[0];
                float dy = node->point[1] - target[1];
                float distance = sqrt( dx*dx + dy*dy);

                if(distance <= distanceTol)
                {
                    ids.push_back(node->id);
                    std::cout << "id = " << node->id << " / distance = " << distance << std::endl;
                }
            }

            // recursively do searchHelper
            // Check : we must search in which area
            int XorY = depth % 2;
            bool selectLowerArea  = (target[XorY] - distanceTol) < node->point[XorY];
            bool selectHigherArea = (target[XorY] + distanceTol) > node->point[XorY];
            if( selectLowerArea )
            {
                searchHelper(target, node->left, depth+1, distanceTol, ids);
            }
            if( selectHigherArea )
            {
                searchHelper(target, node->right, depth+1, distanceTol, ids);
            }
        }
    }


    //--------------------------------------------------------------------------------------//
    // Lesson : Lidar-3-7 : Searching Points in a KD-Tree
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
        // Helper function
        searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}
	

};




