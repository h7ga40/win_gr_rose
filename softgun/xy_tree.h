#ifndef XY_TREE_H
#define XY_TREE_H

#include<stdio.h>
#include<stdlib.h>

struct XY_Tree;

typedef struct xy_node {
	void *key;
	void *value;
	struct xy_node *left;
	struct xy_node *right;
	struct XY_Tree *parent;
	unsigned int tid;
} xy_node;

typedef struct XY_Tree {
	// all fields are private
	int (*Compare) (const void *a, const void *b);
	void (*DestroyKey) (void *a);
	void (*DestroyValue) (void *a);
	void (*DestroyNode) (void *a);
	xy_node root;
} XY_Tree;

void XY_InitTree(struct XY_Tree *tree,
	int (*CompFunc) (const void *, const void *),
	void (*DestFunc) (void *),
	void (*ValueDestFunc) (void *),
	void (*nodeDestFunc) (void *));
void XY_AddTreeNode(struct XY_Tree *, xy_node * node, void *key, void *val);
void XY_DeleteTreeNode(struct XY_Tree *, xy_node *);

void XY_DeleteTree(struct XY_Tree *);

xy_node *XY_NextTreeNode(struct XY_Tree *, xy_node * node);
#define XY_NodeValue(node) ((node)->value)
#define XY_NodeKey(node) ((node)->key)

#endif				// XY_TREE_H
