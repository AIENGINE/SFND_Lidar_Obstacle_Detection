/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point; //point[0] is x point[1] is y
	int id;
	Node* left;
	Node* right;

	Node(const std::vector<float> arr, const int setId)
	:	point(arr), id(setId), left(nullptr), right(nullptr)
	{}
	~Node()= default;
};

struct KdTree
{
	Node* root;

	KdTree() : root(nullptr){}

	void inserPoint(Node** root, std::vector<float>& point, uint depth, int id)
    {
	    if (*root == nullptr)
        {
            *root = new Node(point, id);
        }
        else
        {
            uint current_dim = depth % 2; //selecting x or y index for comparison as depth of the tree when it is traversed
            if (point[current_dim] < (*root)->point[current_dim])
                inserPoint(&((*root)->left), point, depth+1, id);
            else
                inserPoint(&((*root)->right), point, depth+1, id);
        }
    }

	void insert(std::vector<float>& point, int id)
	{
		inserPoint(&root, point, 0, id);
	}

	void searchPoint(Node* kdnode, const std::vector<float>& target, std::vector<int>& ids, uint depth, const float distanceTol)
    {
	    // compare tolerance of target x and y against Node x, y
	    // compute distance if Node x,y is within target x,y with tolerances applied
	    // if distance is less then or equal to given global tolerance push the ids of the points
	    // all the steps are done while traversing the nodes where left nodes are traversed when
	    // dim_point - tolerance < node_point dim and right node is traversed dim_point + tolerance > node_point dim
	    if (kdnode != nullptr)
	    {
            if ((kdnode->point[0]) >= (target[0] - distanceTol) && (kdnode->point[0]) <= (target[0] + distanceTol)
                && (kdnode->point[1]) >= (target[1] - distanceTol) && (kdnode->point[1]) <= (target[1] + distanceTol))
            {
                float target_node_distance = sqrt((kdnode->point[0] - target[0]) * (kdnode->point[0] - target[0]) +
                                                  (kdnode->point[1] - target[1]) * (kdnode->point[1] - target[1]));
                if (target_node_distance < distanceTol)
                    ids.push_back(kdnode->id);
            }

            uint current_dim = depth % 2;
            if (target[current_dim] - distanceTol < kdnode->point[current_dim])
                searchPoint(kdnode->left, target, ids, depth + 1, distanceTol);
            if (target[current_dim] + distanceTol > kdnode->point[current_dim])
                searchPoint(kdnode->right, target, ids, depth + 1, distanceTol);
        }
    }

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(const std::vector<float>& target, const float distanceTol)
	{
		std::vector<int> ids_vector;
		searchPoint(root, target, ids_vector, 0, distanceTol);
		return ids_vector;
	}

	~KdTree()= default;

};




