package org.roboclub.robobuggy.numbers;

/**
 * In mathematics, a group is an algebraic structure consisting of a set of elements together with 
 * an operation that combines any two elements to form a third element. The operation satisfies four
 * conditions called the group axioms, namely closure, associativity, identity and invertibility.
 * @author Trevor Decker
 *
 */
public interface GroupElement extends SetElement{
	//Note the operator must be associative
	public GroupElement operator(GroupElement otherElement);
	
	//The element when used with the operator will result in the other element 
	public GroupElement getIdentity();
	public GroupElement getZero();
	public GroupElement getInverse();
}
