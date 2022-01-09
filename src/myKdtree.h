/* \author Takuya Kobayashi */
#ifndef MYKDTREE_H
#define MYKDTREE_H
#include <iostream>
#include <vector>
#include <string>

//-----------------------------------------------------------------------------------//
// Structure to represent node of kd tree
//-----------------------------------------------------------------------------------//
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
}; // end of struct Node

//-----------------------------------------------------------------------------------//
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
            uint current_dim = depth % points.size(); // 0 or 1
            
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
    // Project : For 3D
    void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
    {
        if(node != nullptr)
        {
            // Check if node is in the box of target torelance
            const int nodeSize   = node->point.size();
            const int targetSize = target.size();
            int coordSize;

            if(targetSize > nodeSize){
                coordSize = nodeSize;
            }else{
                coordSize = targetSize;
            }

            //std::cout << "coordSize = " << coordSize << std::endl;
            bool targetIsInArea = true;
            for(int i=0; i < coordSize; i++)
            {
                targetIsInArea *= (node->point[i] >= (target[i]-distanceTol));
                targetIsInArea *= (node->point[i] <= (target[i]+distanceTol));
            }

            //
            if(targetIsInArea)
            {
                float distance = 0;

                for(int i=0; i < coordSize; i++)
                {
                    float tmpDistance = (node->point[i] - target[i]);
                    distance += tmpDistance * tmpDistance;
                }

                distance = sqrt(distance);

                if(distance <= distanceTol)
                {
                    ids.push_back(node->id);
                    //std::cout << "id = " << node->id << " / distance = " << distance << std::endl;
                }
            }

            // recursively do searchHelper
            // Check : we must search in which area
            int XorYorZ = depth % coordSize;
            bool selectLowerArea  = (target[XorYorZ] - distanceTol) < node->point[XorYorZ];
            bool selectHigherArea = (target[XorYorZ] + distanceTol) > node->point[XorYorZ];

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
}; // end of struct KdTree

#endif




