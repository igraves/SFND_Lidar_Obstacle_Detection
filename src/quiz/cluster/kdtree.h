/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <math.h>


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
};

struct KdTree
{
	Node* root;
	int dimension;

	KdTree()
	: root(NULL)
	{
	    this->dimension = 2;
	}

	KdTree(int dimension)
            : root(NULL)
    {
	    this->dimension = dimension;
    }

	void insert(std::vector<float> point, int id){
	    this->_insert(point, id, &root, false);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		this->_search(target,distanceTol,this->root,0,&ids);
		return ids;
	}

private:
    void _search(std::vector<float> target, float distanceTol, Node* node, int depth, std::vector<int> *ids) {
	    if(node == NULL) {
	        return;
	    }

	    int currentIdx = depth % this->dimension;
	    std::vector<float> currentPoint = node->point;

	    //box containment
	    if(currentPoint[0] > target[0] - distanceTol
	       && currentPoint[0] < target[0] + distanceTol
	       && currentPoint[1] > target[1] - distanceTol
	       && currentPoint[1] < target[1] + distanceTol) {

	        //float distance = std::sqrtf(std::powf(currentPoint[0] - target[0],2) + std::powf(currentPoint[1] - target[1],2));

	        //Generalized euclidean distance

	        float distance = 0.0f;
	        for(int i = 0; i < this->dimension; i++){
	            distance += std::powf(currentPoint[i] - target[i],2.0);
	        }
	        distance = std::sqrtf(distance);


	        if(distance < distanceTol){
	            ids->push_back(node->id);
	        }

	    }

        if(target[currentIdx] - distanceTol < node->point[currentIdx]) {
            //check left
            _search(target,distanceTol,node->left,currentIdx+1,ids);
        }
        if(target[currentIdx] + distanceTol > node->point[currentIdx]) {
            //go right
            _search(target,distanceTol,node->right,currentIdx+1,ids);
        }

	}
    void _insert(std::vector<float> point, int id, Node** parentPtr, int depth) {
        Node* current = *parentPtr;

        if(current == NULL){
            *parentPtr = new Node(point, id);
            return;
        }

        int currentIdx = depth % this->dimension;

        if(current->point[currentIdx] > point[currentIdx]) {
            //go right
            _insert(point,id,&current->left,currentIdx+1);
        } else {
            //go left
            _insert(point,id,&current->right,currentIdx+1);
        }

	}

};




