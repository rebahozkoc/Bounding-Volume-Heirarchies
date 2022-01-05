#include "BVHTree.h"
#include <stack>


void BVHTree::printNode(std::ostream &out, BVHTreeNode *node, int level) {
	if (root == nullptr) return;
	for (int i = 0; i < level; i++) {
		out << "  ";
	}
	if (!node->isLeaf) {
		out << "+ branch || ";
		node->aabb.printAABB(out);
		out << std::endl;
		printNode(out, node->rightChild, level + 1);
		printNode(out, node->leftChild, level + 1);
	}
	else {
		out << "- ";
		if (node->parent) {
			if (node->parent->rightChild == node)
				out << "R ";
			else
				out << "L ";
		}
		out << "- leaf: " << node->name << " || ";
		node->aabb.printAABB(out);
		out << std::endl;
	}
}
std::ostream &operator<<(std::ostream &out, BVHTree &tree) {
	tree.printNode(out, tree.root, 0);
	return out;
}


BVHTree::BVHTree():root(nullptr), map() {

}

BVHTree::~BVHTree() {
	// Iterating over the map using iterator until map end.
	while (!map.empty()) {
		std::unordered_map<std::string, BVHTreeNode*>::iterator it = map.begin();
		while (it != map.end()) {
			// Accessing the key
			std::string name = it->first;
			// Remove node
			(*this).removeBVHMember(name);
			// Reset the iterator because the map has changed
			break;
		}
	}
}

BVHTreeNode* BVHTree::updateParentsAABB(BVHTreeNode* updatedNode) {
	// Updates the parent AABB fields iteratively
	// Returns the root node
	BVHTreeNode* temp = updatedNode;
	while (temp->parent != nullptr) {
		temp->parent->aabb = AABB::join(temp->parent->rightChild->aabb, temp->parent->leftChild->aabb);
		temp = temp->parent;
	}
	return temp;
}

void BVHTree::addBVHMember(AABB objectArea, std::string name) {
	BVHTreeNode* newNode = new BVHTreeNode(objectArea, name, true);
	if (map.empty()) {
		map[name] = newNode;
		root = newNode;
	}
	else {
		if (map.size() == 1) {
			BVHTreeNode* branchNode = new BVHTreeNode(AABB::join(objectArea, root->aabb), "branch", false);
			branchNode->rightChild = root;
			branchNode->leftChild = newNode;
			newNode->parent = branchNode;
			root->parent = branchNode;
			root = branchNode;
			map[name] = newNode;
		}
		else {
			// Find the leaf which will have minimum increase in area with the new node
			BVHTreeNode* existingLeaf = root;
			while(!existingLeaf->isLeaf){
				int increaseInRightTreeSize = AABB::unionArea(newNode->aabb, existingLeaf->rightChild->aabb) - existingLeaf->rightChild->aabb.getArea();
				int increaseInLeftTreeSize = AABB::unionArea(newNode->aabb, existingLeaf->leftChild->aabb) - existingLeaf->leftChild->aabb.getArea();
				if (increaseInRightTreeSize < increaseInLeftTreeSize) {
					existingLeaf = existingLeaf->rightChild;
				}
				else {
					existingLeaf = existingLeaf->leftChild;
				}
			}
			BVHTreeNode* branchNode = new BVHTreeNode(AABB::join(objectArea, existingLeaf->aabb), "branch", false);
			branchNode->leftChild = newNode;
			branchNode->rightChild = existingLeaf;

			// Check which child is the existingLeaf of the old branch parent
			if (existingLeaf->parent->leftChild->name == existingLeaf->name) {
				existingLeaf->parent->leftChild = branchNode;
			}
			else {
				existingLeaf->parent->rightChild = branchNode;
			}

			newNode->parent = branchNode;
			branchNode->parent = existingLeaf->parent;
			existingLeaf->parent = branchNode;
			map[name] = newNode;

			//Adjust parent nodes
			root = updateParentsAABB(newNode);
		}
	}
}
void BVHTree::moveBVHMember(std::string name, AABB newLocation) {
	// Element is not in the tree
	if (map.find(name) == map.end()) {
		std::cout << name << " is not in the BVHTree" << std::endl;
		return;
	}
	
	BVHTreeNode* movedNode = map[name];
	
	if (movedNode->parent == nullptr) {
		movedNode->aabb = newLocation;
		return;
	}

	// parent of the moved node still covers the moved node
	if (AABB::contains(movedNode->parent->aabb, newLocation)) {
		movedNode->aabb = newLocation;
	}
	else {
		removeBVHMember(name);
		addBVHMember(newLocation, name);
	}

}

void BVHTree::removeBVHMember(std::string name) {
	// Element is not in the tree
	if (map.find(name) == map.end()) {
		std::cout << name << " is not in the BVHTree" << std::endl;
		return;
	}
	BVHTreeNode* toRemove = map[name];

	// If the searched node is the only node in the tree resets root
	if (toRemove == root) {
		delete toRemove;
		root = nullptr;
		map.erase(name);
		return;
	}

	// If the parent of the searched node is the root updates the root
	if (toRemove->parent == root) {
		BVHTreeNode* branch = root;
		if (toRemove->parent->rightChild->name == name) {
			root = toRemove->parent->leftChild;
			root->parent = nullptr;
		}
		else {
			root = toRemove->parent->rightChild;
			root->parent = nullptr;
		}
		delete toRemove;
		delete branch;
		map.erase(name);
		return;
	}
	
	// Else removes the node and the parent branch and replaces with the sibling
	BVHTreeNode* sibling;
	BVHTreeNode* branch = toRemove->parent;

	// Find the sibling
	if (branch->leftChild->name == name) {
		sibling = toRemove->parent->rightChild;
	}
	else {
		sibling = toRemove->parent->leftChild;
	}

	sibling->parent = branch->parent;
	// Find which child of parent of the branch is branch and update
	if (branch->parent->rightChild == branch) {
		branch->parent->rightChild = sibling;
	}
	else {
		branch->parent->leftChild = sibling;
	}

	// Calculate AABB objects again
	updateParentsAABB(branch->parent);

	delete toRemove;
	delete branch;
	map.erase(name);
	return;
}

std::vector<std::string> BVHTree::getCollidingObjects(AABB object) {
	// Used the stacks to traverse the binary tree without using recursion
	std::vector<std::string> results;
	std::stack<BVHTreeNode*> nodes;
	nodes.push(root);

	while (!nodes.empty()) {
		BVHTreeNode* node = nodes.top();
		nodes.pop();
		if (node->aabb.collide(object)) {
			if (node->rightChild->aabb.collide(object)) {

				if (node->rightChild->isLeaf) {

					results.push_back(node->rightChild->name);
				}
				else {
					nodes.push(node->rightChild);
				}
			}
			if (node->leftChild->aabb.collide(object)) {
				if (node->leftChild->isLeaf) {
					results.push_back(node->leftChild->name);
				}
				else {
					nodes.push(node->leftChild);
				}
			}
		}
	}
	
	return results;
}