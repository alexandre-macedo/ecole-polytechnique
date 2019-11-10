#include "QuadTree.hpp"
#include <iostream>

QuadTree::QuadTree(std::vector<Atom *> &v, Rectangle &r)
{

    rectangle = r;

    if (v.size() == 0)
    {

	topLeft = 0;
	topRight = 0;
	bottomLeft = 0;
	bottomRight = 0;

	atom = 0;
    }
    else if (v.size() == 1)
    {

	topLeft = 0;
	topRight = 0;
	bottomLeft = 0;
	bottomRight = 0;

	atom = v[0];
    }
    else
    {

	atom = 0;

	// Exercice 2.1 : completer ici
	// Subdivisez le rectangle en quatre sous-rectangles, et construisez les quatre quad-trees associes
	Rectangle tl = rectangle.topLeftRectangle();
	Rectangle tr = rectangle.topRightRectangle();
	Rectangle bl = rectangle.bottomLeftRectangle();
	Rectangle br = rectangle.bottomRightRectangle();
	std::vector<Atom *> atomVectorTl;
	std::vector<Atom *> atomVectorTr;
	std::vector<Atom *> atomVectorBl;
	std::vector<Atom *> atomVectorBr;

	for (std::vector<Atom *>::iterator it = v.begin(); it != v.end(); ++it)
	{
	    if (tl.contains(*it))
		atomVectorTl.push_back(*it);
	    else if (bl.contains(*it))
		atomVectorBl.push_back(*it);
	    else if (br.contains(*it))
		atomVectorBr.push_back(*it);
	    else if (tr.contains(*it))
		atomVectorTr.push_back(*it);
	}

	topLeft = new QuadTree(atomVectorTl, tl);
	topRight = new QuadTree(atomVectorTr, tr);
	bottomLeft = new QuadTree(atomVectorBl, bl);
	bottomRight = new QuadTree(atomVectorBr, br);
	// ...
    }
}

void QuadTree::print(unsigned int offset) const
{

    if (topLeft)
    {

	for (unsigned int i = 0; i < offset; i++)
	    std::cout << "\t";
	std::cout << "topLeft";
	std::cout << std::endl;
	topLeft->print(offset + 1);

	for (unsigned int i = 0; i < offset; i++)
	    std::cout << "\t";
	std::cout << "topRight";
	std::cout << std::endl;
	topRight->print(offset + 1);

	for (unsigned int i = 0; i < offset; i++)
	    std::cout << "\t";
	std::cout << "bottomLeft";
	std::cout << std::endl;
	bottomLeft->print(offset + 1);

	for (unsigned int i = 0; i < offset; i++)
	    std::cout << "\t";
	std::cout << "bottomRight";
	std::cout << std::endl;
	bottomRight->print(offset + 1);
    }
    else if (atom)
    {

	for (unsigned int i = 0; i < offset; i++)
	    std::cout << "\t";
	std::cout << "Atom = " << atom->x << " " << atom->y << std::endl;
    }
}

QuadTree::~QuadTree()
{

    if (topLeft)
	delete topLeft;
    if (topRight)
	delete topRight;
    if (bottomLeft)
	delete bottomLeft;
    if (bottomRight)
	delete bottomRight;
}

void QuadTree::rangeSearch(std::vector<Atom *> &v, Rectangle &r)
{

    // Exercice 2.2 : completer ici
    if (this->rectangle.intersects(r))
    {
	if (this->atom && r.contains(this->atom)) v.push_back(this->atom);
	if (this->topLeft)
	    this->topLeft->rangeSearch(v, r);
	if (this->topRight)
	    this->topRight->rangeSearch(v, r);
	if (this->bottomLeft)
	    this->bottomLeft->rangeSearch(v, r);
	if (this->bottomRight)
	    this->bottomRight->rangeSearch(v, r);
    }
	else return;
}
