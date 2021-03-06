/*
 * -----------------------------------------------------------------
 * red-black tree taken from Emim Martinian's
 * Red-Black Tree C-Code (See http://freshmeat.net/projects/rbtree) 
 *
 * modified for use with xy_lib by Jochen Karrer 02/05/2002 
 * -----------------------------------------------------------------
 */
#include "sgstring.h"
#include <xy_tree.h>

#ifdef TREE_DEBUG
static void
Assert(int assertion, char *error)
{
	if (!assertion) {
		printf("Assertion Failed: %s\n", error);
		exit(-1);
	}
}
#endif

static void
do_nothing(void *v)
{				/* do nothing */
}

/************************************************************************/
/*  FUNCTION:  XY_InitTree 						*/
/*									*/
/*  INPUTS:  All the inputs are names of functions.  CompFunc takes to  */
/*  void pointers to keys and returns 1 if the first arguement is       */
/*  "greater than" the second.   DestFunc takes a pointer to a key and  */
/*  destroys it in the appropriate manner when the node containing that */
/*  key is deleted.  ValueDestFunc is similiar to DestFunc except it    */
/*  recieves a pointer to the value of a node and destroys it.          */
/*  PrintFunc recieves a pointer to the key of a node and prints it.    */
/*  PrintInfo recieves a pointer to the value of a node and prints it.  */
/*  If RBTreePrint is never called the print functions don't have to be */
/*  defined and NullFunction can be used.                               */
/*                                                                      */
/*  OUTPUT:  This function returns a pointer to the newly created       */
/*  red-black tree.                                                     */
/*                                                                      */
/*  Modifies Input: none                                                */
/************************************************************************/

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
	temp = newTree->nilp = &newTree->nil;
	temp->parent = temp->left = temp->right = temp;
	temp->red = 0;
	temp->key = 0;
	temp = newTree->rootp = &newTree->root;
	temp->parent = temp->left = temp->right = newTree->nilp;
	temp->key = 0;
	temp->red = 0;
}

/************************************************************************/
/*  FUNCTION:  LeftRotate 						*/
/*									*/
/*  INPUTS:  This takes a tree so that it can access the appropriate    */
/*           root and nilp pointers, and the node to rotate on.          */
/*									*/
/*  OUTPUT:  None 							*/
/*									*/
/*  Modifies Input: tree, x 						*/
/*									*/
/*  EFFECTS:  Rotates as described in _Introduction_To_Algorithms by    */
/*            Cormen, Leiserson, Rivest (Chapter 14).  Basically this   */
/*            makes the parent of x be to the left of x, x the parent of*/
/*            its parent before the rotation and fixes other pointers   */
/*            accordingly. 						*/
/************************************************************************/

static void
LeftRotate(XY_Tree * tree, xy_node * x)
{
	xy_node *y;
	xy_node *nilp = tree->nilp;

	/*  I originally wrote this function to use the sentinel for */
	/*  nilp to avoid checking for nilp.  However this introduces a */
	/*  very subtle bug because sometimes this function modifies */
	/*  the parent pointer of nilp.  This can be a problem if a */
	/*  function which calls LeftRotate also uses the nilp sentinel */
	/*  and expects the nilp sentinel's parent pointer to be unchanged */
	/*  after calling this function.  For example, when RBDeleteFixUP */
	/*  calls LeftRotate it expects the parent pointer of nilp to be */
	/*  unchanged. */

	y = x->right;
	x->right = y->left;

	if (y->left != nilp)
		y->left->parent = x;	/* used to use sentinel here */
	/* and do an unconditional assignment instead of testing for nilp */

	y->parent = x->parent;

	/* instead of checking if x->parent is the root as in the book, we */
	/* count on the root sentinel to implicitly take care of this case */
	if (x == x->parent->left) {
		x->parent->left = y;
	} else {
		x->parent->right = y;
	}
	y->left = x;
	x->parent = y;

#ifdef DEBUG_ASSERT
	Assert(!tree->nilp->red, "nil not red in LeftRotate");
#endif
}

/*************************************************************************/
/*  FUNCTION:  RightRotate 						 */
/*									 */
/*  INPUTS:  This takes a tree so that it can access the appropriate 	 */
/*           root and nilp pointers, and the node to rotate on. 	 */
/*									 */
/*  OUTPUT:  None 							 */
/*									 */
/*  Modifies Input?: tree, y 						 */
/*									 */
/*  EFFECTS:  Rotates as described in _Introduction_To_Algorithms by     */
/*            Cormen, Leiserson, Rivest (Chapter 14).  Basically this 	 */
/*            makes the parent of x be to the left of x, x the parent of */
/*            its parent before the rotation and fixes other pointers    */
/*            accordingly. 						 */
/*************************************************************************/

static void
RightRotate(XY_Tree * tree, xy_node * y)
{
	xy_node *x;
	xy_node *nilp = tree->nilp;

	/*  I originally wrote this function to use the sentinel for */
	/*  nilp to avoid checking for nilp.  However this introduces a */
	/*  very subtle bug because sometimes this function modifies */
	/*  the parent pointer of nilp.  This can be a problem if a */
	/*  function which calls LeftRotate also uses the nilp sentinel */
	/*  and expects the nilp sentinel's parent pointer to be unchanged */
	/*  after calling this function.  For example, when RBDeleteFixUP */
	/*  calls LeftRotate it expects the parent pointer of nilp to be */
	/*  unchanged. */

	x = y->left;
	y->left = x->right;

	if (nilp != x->right)
		x->right->parent = y;	/*used to use sentinel here */
	/* and do an unconditional assignment instead of testing for nilp */

	/* instead of checking if x->parent is the root as in the book, we */
	/* count on the root sentinel to implicitly take care of this case */
	x->parent = y->parent;
	if (y == y->parent->left) {
		y->parent->left = x;
	} else {
		y->parent->right = x;
	}
	x->right = y;
	y->parent = x;

#ifdef DEBUG_ASSERT
	Assert(!tree->nilp->red, "nil not red in RightRotate");
#endif
}

/****************************************************************************/
/*  FUNCTION:  TreeInsertHelp                                               */
/*                                                                          */
/*  INPUTS:  tree is the tree to insert into and z is the node to insert    */
/*                                                                          */
/*  OUTPUT:  none                                                           */
/*                                                                          */
/*  Modifies Input:  tree, z                                                */
/*                                                                          */
/*  EFFECTS:  Inserts z into the tree as if it were a regular binary tree   */
/*            using the algorithm described in _Introduction_To_Algorithms_ */
/*            by Cormen et al.  This funciton is only intended to be called */
/*            by the RBTreeInsert function and not by the user              */
/****************************************************************************/

static void
TreeInsertHelp(XY_Tree * tree, xy_node * z)
{
	/*  This function should only be called by InsertRBTree (see above) */
	xy_node *x;
	xy_node *y;
	xy_node *nilp = tree->nilp;

	z->left = z->right = nilp;
	y = tree->rootp;
	x = tree->rootp->left;
	while (x != nilp) {
		y = x;
		if (tree->Compare(x->key, z->key) > 0) {	/* x.key > z.key */
			x = x->left;
		} else {	/* x,key <= z.key */
			x = x->right;
		}
	}
	z->parent = y;
	if ((y == tree->rootp) || (tree->Compare(y->key, z->key) > 0)) {	/* y.key > z.key */
		y->left = z;
	} else {
		y->right = z;
	}

#ifdef DEBUG_ASSERT
	Assert(!tree->nilp->red, "nilp not red in TreeInsertHelp");
#endif
}

/*  Before calling Insert RBTree the node x should have its key set */

/***********************************************************************/
/*  FUNCTION:  RBTreeInsert */
 /**/
/*  INPUTS:  tree is the red-black tree to insert a node which has a key */
/*           pointed to by key and value pointed to by value.  */
     /**/
/*  OUTPUT:  This function returns a pointer to the newly inserted node */
/*           which is guarunteed to be valid until this node is deleted. */
/*           What this means is if another data structure stores this */
/*           pointer then the tree does not need to be searched when this */
/*           is to be deleted. */
     /**/
/*  Modifies Input: tree */
     /**/
/*  EFFECTS:  Creates a node node which contains the appropriate key and */
/*            value pointers and inserts it into the tree. */
/***********************************************************************/
    void
XY_AddTreeNode(XY_Tree * tree, xy_node * x, void *key, void *value)
{
	xy_node *y;

	x->key = key;
	x->value = value;

	TreeInsertHelp(tree, x);
	x->red = 1;

	while (x->parent->red) {	/* use sentinel instead of checking for root */
		if (x->parent == x->parent->parent->left) {
			y = x->parent->parent->right;
			if (y->red) {
				x->parent->red = 0;
				y->red = 0;
				x->parent->parent->red = 1;
				x = x->parent->parent;
			} else {
				if (x == x->parent->right) {
					x = x->parent;
					LeftRotate(tree, x);
				}
				x->parent->red = 0;
				x->parent->parent->red = 1;
				RightRotate(tree, x->parent->parent);
			}
		} else {	/* case for x->parent == x->parent->parent->right */
			y = x->parent->parent->left;
			if (y->red) {
				x->parent->red = 0;
				y->red = 0;
				x->parent->parent->red = 1;
				x = x->parent->parent;
			} else {
				if (x == x->parent->left) {
					x = x->parent;
					RightRotate(tree, x);
				}
				x->parent->red = 0;
				x->parent->parent->red = 1;
				LeftRotate(tree, x->parent->parent);
			}
		}
	}
	tree->rootp->left->red = 0;

#ifdef DEBUG_ASSERT
	Assert(!tree->nilp->red, "nil not red in RBTreeInsert");
	Assert(!treep->root->red, "root not red in RBTreeInsert");
#endif
}

xy_node *
XY_CreateTreeNode(XY_Tree * tree, void *key, void *value)
{
	xy_node *node = (xy_node *) sg_new(xy_node);
	if (!node)
		return NULL;
	XY_AddTreeNode(tree, node, key, value);
	return node;
}

/***************************************************************************/
/*  FUNCTION:  TreeSuccessor  						   */
/*									   */
/*    INPUTS:  tree is the tree in question, and x is the node we want the */
/*             the successor of. 					   */
/*									   */
/*    OUTPUT:  This function returns the successor of x or NULL if no      */
/*             successor exists. 					   */
/*									   */
/*    Modifies Input: none 						   */
/*									   */
/*    Note:  uses the algorithm in _Introduction_To_Algorithms_            */
/***************************************************************************/

static xy_node *
TreeSuccessor(XY_Tree * tree, xy_node * x)
{
	xy_node *y;
	xy_node *nilp = tree->nilp;
	xy_node *rootp = tree->rootp;

	if (nilp != (y = x->right)) {	/* assignment to y is intentional */
		while (y->left != nilp) {	/* returns the minium of the right subtree of x */
			y = y->left;
		}
		return (y);
	} else {
		y = x->parent;
		while (x == y->right) {	/* sentinel used instead of checking for nilp */
			x = y;
			y = y->parent;
		}
		if (y == rootp)
			return (nilp);
		return (y);
	}
}

#if 0
/***********************************************************************/
/*  FUNCTION:  Treepredecessor  */
 /**/
/*    INPUTS:  tree is the tree in question, and x is the node we want the */
/*             the predecessor of. */
     /**/
/*    OUTPUT:  This function returns the predecessor of x or NULL if no */
/*             predecessor exists. */
     /**/
/*    Modifies Input: none */
     /**/
/*    Note:  uses the algorithm in _Introduction_To_Algorithms_ */
/***********************************************************************/
static xy_node *
TreePredecessor(XY_Tree * tree, xy_node * x)
{
	xy_node *y;
	xy_node *nilp = tree->nilp;
	xy_node *root = tree->rootp;

	if (nilp != (y = x->left)) {	/* assignment to y is intentional */
		while (y->right != nilp) {	/* returns the maximum of the left subtree of x */
			y = y->right;
		}
		return (y);
	} else {
		y = x->parent;
		while (x == y->left) {
			if (y == root)
				return (nilp);
			x = y;
			y = y->parent;
		}
		return (y);
	}
}

#endif
/***********************************************************************/
/*  FUNCTION:  TreeDestHelper */
 /**/
/*    INPUTS:  tree is the tree to destroy and x is the current node */
     /**/
/*    OUTPUT:  none  */
     /**/
/*    EFFECTS:  This function recursively destroys the nodes of the tree */
/*              postorder using the DestroyKey and DestroyInfo functions. */
     /**/
/*    Modifies Input: tree, x */
     /**/
/*    Note:    This function should only be called by RBTreeDestroy */
/***********************************************************************/
    static void
TreeDestHelper(XY_Tree * tree, xy_node * x)
{
	xy_node *nilp = tree->nilp;
	if (x != nilp) {
		TreeDestHelper(tree, x->left);
		TreeDestHelper(tree, x->right);
		tree->DestroyKey(x->key);
		tree->DestroyValue(x->value);
		tree->DestroyNode(x);
	}
}

/***********************************************************************/
/*  FUNCTION:  RBTreeDestroy */
 /**/
/*    INPUTS:  tree is the tree to destroy */
     /**/
/*    OUTPUT:  none */
     /**/
/*    EFFECT:  Destroys the key and frees memory */
     /**/
/*    Modifies Input: tree */
     /**/
/***********************************************************************/
    void
XY_DeleteTree(XY_Tree * tree)
{
	TreeDestHelper(tree, tree->rootp->left);
}

/************************************************************************/
/*  FUNCTION:  RBFind 							*/
/*									*/
/*    INPUTS:  tree is the tree to print and q is a pointer to the key  */
/*             we are searching for 					*/
/*									*/
/*    OUTPUT:  returns the a node with key equal to q.  If there are 	*/
/*             multiple nodes with key equal to q this function returns */
/*             the one highest in the tree 				*/
/*									*/
/*    Modifies Input: none 						*/
/*									*/
/************************************************************************/

xy_node *
XY_FindTreeNode(XY_Tree * tree, void *q)
{
	xy_node *x = tree->rootp->left;
	xy_node *nilp = tree->nilp;
	int compVal;
	if (x == nilp)
		return (0);
	compVal = tree->Compare(x->key, q);
	while (0 != compVal) {	/*assignemnt */
		if (compVal > 0) {	/* x->key > q */
			x = x->left;
		} else {
			x = x->right;
		}
		if (x == nilp)
			return (0);
		compVal = tree->Compare(x->key, q);
	}
	return (x);
}

xy_node *
XY_FindLeftTreeNode(XY_Tree * tree, void *q)
{
	xy_node *x = tree->rootp->left;
	xy_node *nilp = tree->nilp;
	xy_node *last = NULL;
	int compVal;
	if (x == nilp)
		return (0);
	compVal = tree->Compare(q, x->key);
	while (0 != compVal) {	/*assignemnt */
		if (compVal > 0) {	/* x->key > q */
			last = x;
			x = x->right;
			if (x == nilp) {
				return last;
			}
		} else {	/* < 0 */

			x = x->left;
			if (x == nilp) {
				return last;
			}
		}
		compVal = tree->Compare(q, x->key);
	}
	return x;
}

xy_node *
XY_FirstTreeNode(XY_Tree * tree)
{
	xy_node *x = tree->rootp->left;
	if (x == tree->nilp)
		return (0);
	while (x->left && (x->left != tree->nilp)) {
		x = x->left;
	}
	return x;
}

/*
 * --------------------------------
 * Find Successor 
 * --------------------------------
 */
xy_node *
XY_NextTreeNode(XY_Tree * tree, xy_node * x)
{
	xy_node *node = TreeSuccessor(tree, x);
	if (node == tree->nilp) {
		return NULL;
	}
	return node;
}

/*************************************************************************/
/*  FUNCTION:  RBDeleteFixUp 						 */
/*									 */
/*    INPUTS:  tree is the tree to fix and x is the child of the spliced */
/*             out node in RBTreeDelete. 				 */
/*									 */
/*    OUTPUT:  none 							 */
/*									 */
/*    EFFECT:  Performs rotations and changes colors to restore red-black*/
/*             properties after a node is deleted 			 */
/*									 */
/*    Modifies Input: tree, x 						 */
/*									 */
/*    The algorithm from this function is from _Introduction_To_Algorithms_ */
/**************************************************************************/

static void
RBDeleteFixUp(XY_Tree * tree, xy_node * x)
{
	xy_node *root = tree->rootp->left;
	xy_node *w;

	while ((!x->red) && (root != x)) {
		if (x == x->parent->left) {
			w = x->parent->right;
			if (w->red) {
				w->red = 0;
				x->parent->red = 1;
				LeftRotate(tree, x->parent);
				w = x->parent->right;
			}
			if ((!w->right->red) && (!w->left->red)) {
				w->red = 1;
				x = x->parent;
			} else {
				if (!w->right->red) {
					w->left->red = 0;
					w->red = 1;
					RightRotate(tree, w);
					w = x->parent->right;
				}
				w->red = x->parent->red;
				x->parent->red = 0;
				w->right->red = 0;
				LeftRotate(tree, x->parent);
				x = root;	/* this is to exit while loop */
			}
		} else {	/* the code below is has left and right switched from above */
			w = x->parent->left;
			if (w->red) {
				w->red = 0;
				x->parent->red = 1;
				RightRotate(tree, x->parent);
				w = x->parent->left;
			}
			if ((!w->right->red) && (!w->left->red)) {
				w->red = 1;
				x = x->parent;
			} else {
				if (!w->left->red) {
					w->right->red = 0;
					w->red = 1;
					LeftRotate(tree, w);
					w = x->parent->left;
				}
				w->red = x->parent->red;
				x->parent->red = 0;
				w->left->red = 0;
				RightRotate(tree, x->parent);
				x = root;	/* this is to exit while loop */
			}
		}
	}
	x->red = 0;

#ifdef DEBUG_ASSERT
	Assert(!tree->nilp->red, "nil not black in RBDeleteFixUp");
#endif
}

/************************************************************************/
/*  FUNCTION:  RBDelete 						*/
/*									*/
/*    INPUTS:  tree is the tree to delete node z from 			*/
/*									*/
/*    OUTPUT:  none 							*/
/*									*/
/*    EFFECT:  Deletes z from tree and frees the key and value of z 	*/
/*             using DestoryKey and DestoryInfo.  Then calls 		*/
/*             RBDeleteFixUp to restore red-black properties 		*/
/*									*/
/*    Modifies Input: tree, z 						*/
/*									*/
/*    The algorithm from this function is from 				*/
/*     _Introduction_To_Algorithms_ 					*/
/************************************************************************/

void
XY_DeleteTreeNode(XY_Tree * tree, xy_node * z)
{
	xy_node *y;
	xy_node *x;
	xy_node *nilp = tree->nilp;
	xy_node *root = tree->rootp;

	y = ((z->left == nilp) || (z->right == nilp)) ? z : TreeSuccessor(tree, z);
	x = (y->left == nilp) ? y->right : y->left;
	if (root == (x->parent = y->parent)) {	/* assignment of y->p to x->p is intentional */
		root->left = x;
	} else {
		if (y == y->parent->left) {
			y->parent->left = x;
		} else {
			y->parent->right = x;
		}
	}
	if (y != z) {		/* y should not be nilp in this case */

#ifdef DEBUG_ASSERT
		Assert((y != tree->nilp), "y is nilp in RBDelete\n");
#endif
		/* y is the node to splice out and x is its child */

		if (!(y->red))
			RBDeleteFixUp(tree, x);

		tree->DestroyKey(z->key);
		tree->DestroyValue(z->value);
		y->left = z->left;
		y->right = z->right;
		y->parent = z->parent;
		y->red = z->red;
		z->left->parent = z->right->parent = y;
		if (z == z->parent->left) {
			z->parent->left = y;
		} else {
			z->parent->right = y;
		}
		tree->DestroyNode(z);
	} else {
		tree->DestroyKey(y->key);
		tree->DestroyValue(y->value);
		if (!(y->red))
			RBDeleteFixUp(tree, x);
		tree->DestroyNode(y);
	}

#ifdef DEBUG_ASSERT
	Assert(!tree->nilp->red, "nilp not black in RBDelete");
#endif
}
