using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class KDTree
{
    public class Node
    {
        public Node left;
        public Node right;
        public double[] data;
        public bool isLeaf;
        public Node(double[] _data)
        {
            data = _data;
        }
    }
    // k represents the num dimensions
    // extraData represents the num of floats at the end that should be ignored
    private int k;
    private int extraData;
    private List<double[]> values;
    private Node root;
    private double currentBestDist;
    private Node closest;
    public KDTree (int _k = 30, int _extraData = 2)
    {
        k = _k;
        extraData = _extraData;
        values = new List<double[]>();
    }
   public void Add(double[] entry)
    {
        values.Add(entry);
    }


    public void Build()
    {
        root = recursiveBuild(0, values);
        values = null;
    }
    private Node recursiveBuild(int depth , List<double[]> values)
    {
        if (values.Count == 0)
            return null;
        int axis = depth % k;
        values.Sort(delegate (double[] x, double[] y)
        {
            return x[axis].CompareTo(y[axis]);
        });
        int median_idx = values.Count / 2;
        Node newNode = new Node(values[median_idx]);
        int numValuesAfterMedian = values.Count - (median_idx + 1);
        List<double[]> valuesLeft = values.GetRange(0, median_idx);
        List<double[]> valuesRight = values.GetRange(median_idx + 1, numValuesAfterMedian);
        newNode.left = recursiveBuild(depth + 1, valuesLeft);
        newNode.right = recursiveBuild(depth + 1, valuesRight);
        // newNode.isLeaf = newNode.left == null && newNode.right == null;
        //if (!newNode.isLeaf && (newNode.left == null || newNode.right == null))
        //{
        //    Debug.Log("CORNER CASE FOUND! ");
        //}
        return newNode;
    }

    public double[] nnSearch(float[] searchVector, int depth = 0 )
    {
        closest = null;
        currentBestDist = double.PositiveInfinity;
        recursiveNNSearch(root, searchVector, depth);
        return closest.data;
    }

    private void recursiveNNSearch(Node node, float[] searchVector, int depth) 
    {
        if (node == null)
            return;
        int axis = depth % k;
        // if the bounding box is too far, do nothing 
        //if (closest != null && Math.Pow(node.data[axis] - searchVector[axis], 2) > currentBestDist)
        //    return;
        double dist = distanceBetween( node.data, searchVector);
        if (closest == null || dist < currentBestDist)
        {
            closest = node;
            currentBestDist = dist;
        }
        if (searchVector[axis] < node.data[axis])
        {
            // search left first
            recursiveNNSearch(node.left, searchVector, depth + 1);
            // do the weird hypersphere thing wikipedia talked about lol
            // ok but really: 
            // If the distance between the node's value at the splitting dimension and the search vector's va
            if (distBetweenAtAxis(node.data, searchVector, axis) < currentBestDist)
            {
                recursiveNNSearch(node.right, searchVector, depth + 1);
            }
        } else
        {
            // search right first
            recursiveNNSearch(node.right, searchVector, depth + 1);
            if (distBetweenAtAxis(node.data, searchVector, axis) < currentBestDist)
            {
                recursiveNNSearch(node.left, searchVector, depth + 1);
            }
        }
    }
    private double distBetweenAtAxis(double[] a, float[] b, int axis)
    {
        return Math.Pow(a[axis] - b[axis], 2);
    }
    private double distanceBetween(double[] a, float[] b)
    {
        // use squared euclidan distance to avoid having to calculate square roots
        double answer = 0;
        for (int i = 0; i < k; i++)
        {
            answer += Math.Pow(a[i] - b[i], 2);
        }
        return answer;
    }

    public double[] bruteForceSearch(float[] searchVector)
    {
        closest = null;
        currentBestDist = double.PositiveInfinity;

        recursiveBruteForceSearch(root, searchVector);
        return closest.data;
    }
    private void recursiveBruteForceSearch(Node node, float[] searchVector)
    {
        if (node == null)
            return;
        double dist = distanceBetween( node.data, searchVector);
        if (closest == null || dist < currentBestDist)
        {
            closest = node;
            currentBestDist = dist;
        }
        recursiveBruteForceSearch(node.left , searchVector);
        recursiveBruteForceSearch(node.right, searchVector);
    }
}
