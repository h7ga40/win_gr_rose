#include <xy_tree.h>
#include <windows.h>

static void
do_nothing(void *v)
{				/* do nothing */
}

void
XY_InitTree(struct XY_Tree *newTree,
	    int (*CompFunc) (const void *, const void *),
	    void (*DestFunc) (void *), void (*ValueDestFunc) (void *), void (*nodeDestFunc) (void *)
    )
{
	xy_node *temp;

	newTree->Compare = CompFunc;
	newTree->DestroyKey = DestFunc;
	if (!newTree->DestroyKey) {
		newTree->DestroyKey = do_nothing;
	}
	newTree->DestroyValue = ValueDestFunc;
	if (!newTree->DestroyValue) {
		newTree->DestroyValue = do_nothing;
	}
	newTree->DestroyNode = nodeDestFunc;
	if (!newTree->DestroyNode) {
		newTree->DestroyNode = do_nothing;
	}

	/*  see the comment in the XY_Tree structure in red_black_tree.h */
	/*  for information on nilp and root */
	temp = &newTree->root;
	temp->parent = newTree;
	temp->left = temp->right = temp;
	temp->key = 0;
}

void
XY_AddTreeNode(XY_Tree * tree, xy_node * x, void *key, void *value)
{
	if (x->parent != 0)
		/*DebugBreak();*/
		return;

	x->key = key;
	x->value = value;
	x->parent = tree;
	x->right = x;
	x->left = x;
	x->tid = GetCurrentThreadId();

	xy_node *node = tree->root.right;
	xy_node *pos = tree->root.left;
	while (node != &tree->root) {
		if (tree->Compare(node->key, x->key)) {
			pos = node->left;
			break;
		}
		node = node->right;
	}

#if 0
	x->left = pos->left;
	x->right = pos;
	pos->left->right = x;
	pos->left = x;
#else
	x->left = pos;
	x->right = pos->right;
	pos->right->left = x;
	pos->right = x;
#endif
}

int
XY_IsEmpty(XY_Tree *tree)
{
	if (tree->root.left == &tree->root) {
		return 1;
	}
	return 0;
}

void
XY_DeleteTree(XY_Tree * tree)
{
	while (XY_IsEmpty(tree) != 0) {
		XY_DeleteTreeNode(tree, tree->root.left);
	}
}

/*
 * --------------------------------
 * Find Successor 
 * --------------------------------
 */
xy_node *
XY_NextTreeNode(XY_Tree * tree, xy_node * x)
{
	if (x->parent != tree) DebugBreak();

	if (&tree->root == x->right)
		return 0;

	return x->right;
}

void
XY_DeleteTreeNode(XY_Tree * tree, xy_node * z)
{
	if (z->parent != tree) DebugBreak();

	z->right->left = z->left;
	z->left->right = z->right;
	z->parent = 0;
	z->right = 0;
	z->left = 0;

	tree->DestroyKey(z->key);
	tree->DestroyValue(z->value);
	tree->DestroyNode(z);

	z->key = 0;
	z->value = 0;
}
