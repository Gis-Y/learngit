#pragma once
#include <cassert>
#include <iostream>

template<typename TreeType>
void populateAlphabet(TreeType& alphaTree, bool duplicateVowels = false, bool verbose = false)
{
	// create a child iterator
	typename TreeType::iterator it;  

	// insert first node, A
	it = alphaTree.insert(Alpha('A'));
	assert(it != alphaTree.end() && it->getLetter() == 'A' );
	if (duplicateVowels) {
		// try to insert another A.  Should fail for tree & unique_tree
		it = alphaTree.insert(Alpha('A'));
		if ( it == alphaTree.end() && verbose)
			std::cout << alphaTree.get()->getLetter() << ": Couldn't insert second A." << std::endl;
	}

	// insert D and E under A
	it = alphaTree.begin();
	assert(it != alphaTree.end() && it->getLetter() == 'A');
	typename TreeType::iterator child_it = it.node()->insert(Alpha('D'));
	assert(child_it != it.node()->end() && child_it->getLetter() == 'D');  
	it.node()->insert(Alpha('E'));


	it = it.node()->begin();
	// insert  J under D
	it.node()->insert(Alpha('J'));
	// insert K under D and remember inserted node
	child_it = it.node()->insert(Alpha('K'));
	assert(child_it != it.node()->end() && child_it->getLetter() == 'K');
	// insert R and S under K
	child_it.node()->insert(Alpha('R'));
	child_it.node()->insert(Alpha('S'));

	// increment it (now at D) to point to E
	++it;
	// insert L under E
	it.node()->insert(Alpha('L'));

	it = alphaTree.insert(Alpha('B'));
	// insert second E and F under B
	if (duplicateVowels) {
		child_it = it.node()->insert(Alpha('E'));  // should fail for unique_tree
		if ( child_it == it.node()->end() && verbose)
			std::cout << alphaTree.get()->getLetter() << ": Couldn't insert second E." << std::endl;
	}
	child_it = it.node()->insert(Alpha('F'));
	// insert M under F
	it = child_it;
	child_it = it.node()->insert(Alpha('M'));
	// insert T and U under M
	child_it.node()->insert(Alpha('T'));
	child_it.node()->insert(Alpha('U'));

	// insert N under F  (it still points to F)
	child_it = it.node()->insert(Alpha('N'));
	// insert second U and V under N
	if (duplicateVowels) {
		if ( child_it.node()->insert(Alpha('U')) == child_it.node()->end() && verbose) // should fail for unique_tree
			std::cout << alphaTree.get()->getLetter() << ": Couldn't insert second U." << std::endl;
	}

	child_it.node()->insert(Alpha('V'));

	it = alphaTree.insert(Alpha('C'));
	// insert G and H under C
	it.node()->insert(Alpha('G'));
	child_it = it.node()->insert(Alpha('H'));
	// insert O under H
	it = child_it;
	child_it = it.node()->insert(Alpha('O'));
	if (duplicateVowels) {
		// try to insert another O
		child_it = it.node()->insert(Alpha('O')); // should fail for tree/unique_tree
		if ( child_it == it.node()->end() && verbose)
			std::cout << alphaTree.get()->getLetter() << ": Couldn't insert second O." << std::endl;
	}

	child_it = it.node()->begin();
	assert(child_it != it.node()->end() && child_it->getLetter() == 'O');
	// insert W under O
	child_it.node()->insert(Alpha('W'));
	// insert P under H
	it.node()->insert(Alpha('P'));

	// insert I under C using parent of H (which is C)
	child_it = it.node()->parent()->insert(Alpha('I'));
	assert(child_it->getLetter() == 'I');
	// insert second I under I
	it = child_it;
	if (duplicateVowels) {
		child_it = it.node()->insert(Alpha('I'));  // should fail for unique tree
		if ( child_it == it.node()->end() && verbose)
			std::cout << alphaTree.get()->getLetter() << ": Couldn't insert second I." << std::endl;
	}

	// insert Q under original I
	child_it = it.node()->insert(Alpha('Q'));
	// insert X under Q
	it = child_it;
	child_it = it.node()->insert(Alpha('X'));
	// insert Y and Z under X
	child_it.node()->insert(Alpha('Y'));
	child_it.node()->insert(Alpha('Z'));

  if (verbose)
	  std::cout << std::endl << std::endl;
}


