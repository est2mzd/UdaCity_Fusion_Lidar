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
    // self data
	std::vector<float> point; // point of self
	int id;      // id of point of self in cloud array

    // child data
	Node* left;  // left child node
	Node* right; // right child node

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
    // inputs from parent is point and id_in_point_cloud
    void insertHelper(Node** node, uint depth, std::vector<float> point, int id_in_point_cloud)
    {
        // If tree is empty.
        if(*node == nullptr)
        {
            // if left or right child node is not created or root is not created ,
            // create new node, and stop this function

            // *node is Address of the Real Object
            // * is de-reference
            *node = new Node(point, id_in_point_cloud);
        }
        else
        {
            // Calculate current dim
            uint current_dim = depth % point.size(); // 0 or 1 or 2
            
            if(point[current_dim] < ((*node)->point[current_dim])  )
            {
                insertHelper(&((*node)->left), depth+1, point, id_in_point_cloud);
            }
            else
            {
                insertHelper(&((*node)->right), depth+1, point, id_in_point_cloud);
            }
        }
        //std::cout << "id_in_point_cloud = " << id_in_point_cloud << std::endl;
    }

    //--------------------------------------------------------------------------------------//
    // Lesson : Lidar-3-6 : Insert Points
	void insert(std::vector<float> point, int id_in_point_cloud)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
        insertHelper(&root, 0, point, id_in_point_cloud);
	}

    //--------------------------------------------------------------------------------------//
    // Project : For 3D
    void searchHelper(std::vector<float> target_point, Node* node, int depth, float distanceTol, std::vector<int>& nearest_point_ids)
    {
        if(node != nullptr)
        {
            // Check if node is in the box of target_point torelance
            const int coordSize = target_point.size();

            bool targetIsInArea = true;
            for(int i=0; i < coordSize; i++)
            {
                targetIsInArea *= (node->point[i] >= (target_point[i] - distanceTol));
                targetIsInArea *= (node->point[i] <= (target_point[i] + distanceTol));
            }

            //
            if(targetIsInArea)
            {
                float distance = 0;

                for(int i=0; i < coordSize; i++)
                {
                    float tmpDistance = (node->point[i] - target_point[i]);
                    distance += tmpDistance * tmpDistance;
                }

                distance = sqrt(distance);

                if(distance <= distanceTol)
                {
                    nearest_point_ids.push_back(node->id);
                }
            }

            // recursively do searchHelper
            // Check : we must search in which area
            int XorYorZ = depth % coordSize;
            bool selectLowerArea  = (target_point[XorYorZ] - distanceTol) < node->point[XorYorZ];
            bool selectHigherArea = (target_point[XorYorZ] + distanceTol) > node->point[XorYorZ];

            if( selectLowerArea )
            {
                searchHelper(target_point, node->left, depth + 1, distanceTol, nearest_point_ids);
            }

            if( selectHigherArea )
            {
                searchHelper(target_point, node->right, depth + 1, distanceTol, nearest_point_ids);
            }
        }
    }

    //--------------------------------------------------------------------------------------//
    // Lesson : Lidar-3-7 : Searching Points in a KD-Tree
	// return a list of point ids in the tree that are within distance of target_point
	std::vector<int> search(std::vector<float> target_point, float distanceTol)
	{
		std::vector<int> nearest_point_ids;
        // Helper function
        searchHelper(target_point, root, 0, distanceTol, nearest_point_ids);
		return nearest_point_ids;
	}
}; // end of struct KdTree

#endif




