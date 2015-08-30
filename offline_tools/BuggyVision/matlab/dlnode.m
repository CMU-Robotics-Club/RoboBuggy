%based on sample from mathworks 

classdef dlnode < handle
    %dlnode a class to represent a doubly-linked list node.
    %Link multiple dlnode objects togethoer to create linked lists
    properties
        X
        Y
    end
    properties(SetAccess = private)
        Next = dlnode.empty;
        Prev = dlnode.empty;
    end
    
    methods
        function node = dlnode(X,Y)
            % Construct a dlnode object.
            if nargin > 0
                node.X = X;
                node.Y = Y;
            end
        end
        
        function [X,Y] = getData(node)
           X = node.X; 
           Y = node.Y;
        end
        
        function printList(head)
            %TODO
        end
        
        
        function insertAfter(newNode,nodeBefore)
            %Insert newNode after nodeBefore.
            removeNode(newNode);
            newNode.Next = nodeBefore.Next;
            newNode.Prev = nodeBefore;
            if ~isempty(nodeBefore.Next)
                nodeBefore.Next.Prev = newNode;
            end
            nodeBefore.Next = newNode;
        end
        
        function insertBefore(newNode,nodeAfter)
            %Insert newNode before nodeAfter
            removeNode(newNode);
            newNode.next = nodeAfter;
            newNode.Prev = nodeAfter.Prev;
            if ~isempty(nodeAfter.Prev)
                nodeAfter.Prev.Next = newNode;
            end
            nodeAfter.Prev = newNode;
        end
        
        function removeNode(node)
            %Remove a node from a linked list
            if ~isscalar(node)
                error('Input must be scalar')
            end
            prevNode = node.Prev;
            nextNode = node.Next;
            if ~isempty(prevNode)
                prevNode.Next = nextNode;
            end
            if ~isempty(nextNode)
                nextNode.Prev = prevNode;
            end
            node.Next = dlnode.empty;
            node.Prev = dlnode.empty;
        end
        
        function clearList(node)
            % Clear the list before
            % clearing list variable
            prev = node.Prev;
            next = node.Next;
            removeNode(node)
            while ~isempty(next)
                node = next;
                next = node.Next;
                removeNode(node);
            end
            while ~isempty(prev)
                node = prev;
                prev = node.Prev;
                removeNode(node)
            end
        end
    end%methods
    
    methods (Access = private)
        function delete(node)
            %Delete allnodes 
            clearList(node)
        end
    end %private methods
end %classdef