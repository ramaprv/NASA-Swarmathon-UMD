/*
    Copyright (c) 2015, Damian Barczynski <daan.net@wp.eu>
    Following tool is licensed under the terms and conditions of the ISC license.
    For more information visit https://opensource.org/licenses/ISC.
*/
#ifndef __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__
#define __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__

#include <vector>
#include <functional>
#include <set>
#include <queue>
#include <vector>
#include <algorithm>
#include <chrono>
#include <map>
#include "Point.h"
#include "MapController.h"

namespace AStar
{
    struct Vec2i
    {
        int x, y;

        bool operator == (const Vec2i& coordinates_);
    };

    using uint = unsigned int;
    using HeuristicFunction = std::function<uint(Vec2i, Vec2i)>;
    using CoordinateList = std::vector<Vec2i>;

    struct Node
    {
        uint G, H;
        Vec2i coordinates;
        Node *parent;

        Node(Vec2i coord_, Node *parent_ = nullptr);
        uint getScore();
    };

    struct prioritizedNode {
      Node* qNode;
      prioritizedNode(Node* _qNode):
        qNode(_qNode)
      {
      }

      inline bool operator <(const prioritizedNode& other) const
      {
        return qNode->getScore() < other.qNode->getScore();
      }
    };

    using NodeSet = std::set<Node*>;
    using nodeMap = std::map<Point, Vec2i>;
    std::priority_queue<prioritizedNode> nodeQueue;

    class Generator
    {
        bool detectCollision(Vec2i coordinates_);
        Node* findNodeOnList(NodeSet& nodes_, Vec2i coordinates_);
        void releaseNodes(NodeSet& nodes_);

    public:
        Generator();
        void setWorldSize(Vec2i worldSize_);
        void setDiagonalMovement(bool enable_);
        void setHeuristic(HeuristicFunction heuristic_);
        bool findPath();
        void addCollision(Vec2i coordinates_);
        void removeCollision(Vec2i coordinates_);
        void clearCollisions();
        bool rejectWithMap(Vec2i coordinates_);
        bool inClosedMap(Vec2i coord);
        void init(Vec2i src, Vec2i tar, mapObj* ptr);
        CoordinateList getPath();
        void updateParentToMap(Vec2i currentPt, Vec2i parentPt);
        std::map<Point, mapValue>* mapObjPtr;
        Vec2i source_;
        Vec2i target_;
        Node *current;
        nodeMap closedMap;
        Point subOptimalPoint;
        CoordinateList path;


    private:
        HeuristicFunction heuristic;
        CoordinateList direction, walls;
        Vec2i worldSize;
        uint directions;
    };

    class Heuristic
    {
        static Vec2i getDelta(Vec2i source_, Vec2i target_);

    public:
        static uint manhattan(Vec2i source_, Vec2i target_);
        static uint euclidean(Vec2i source_, Vec2i target_);
        static uint octagonal(Vec2i source_, Vec2i target_);
    };
}

#endif // __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__
