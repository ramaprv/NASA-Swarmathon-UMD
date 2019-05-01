#include "AStar.hpp"

using namespace std::placeholders;

bool AStar::Vec2i::operator == (const Vec2i& coordinates_)
{
    return (x == coordinates_.x && y == coordinates_.y);
}

AStar::Vec2i operator + (const AStar::Vec2i& left_, const AStar::Vec2i& right_)
{
    return{ left_.x + right_.x, left_.y + right_.y };
}

AStar::Node::Node(Vec2i coordinates_, Node *parent_)
{
    parent = parent_;
    coordinates = coordinates_;
    G = H = 0;
}

AStar::uint AStar::Node::getScore()
{
    return G + H;
}

AStar::Generator::Generator()
{
    setDiagonalMovement(True);
    setHeuristic(&Heuristic::manhattan);

    direction = {
        { 0, 1 }, { 1, 0 }, { 0, -1 }, { -1, 0 },
        { -1, -1 }, { 1, 1 }, { -1, 1 }, { 1, -1 }
    };
}

void AStar::Generator::setWorldSize(Vec2i worldSize_)
{
    worldSize = worldSize_;
}

void AStar::Generator::setDiagonalMovement(bool enable_)
{
    directions = (enable_ ? 8 : 4);
}

void AStar::Generator::setHeuristic(HeuristicFunction heuristic_)
{
    heuristic = std::bind(heuristic_, _1, _2);
}

void AStar::Generator::addCollision(Vec2i coordinates_)
{
    walls.push_back(coordinates_);
}

void AStar::Generator::removeCollision(Vec2i coordinates_)
{
    auto it = std::find(walls.begin(), walls.end(), coordinates_);
    if (it != walls.end()) {
        walls.erase(it);
    }
}

void AStar::Generator::clearCollisions()
{
    walls.clear();
}

bool AStar::Generator::findPath()
{
    // Node *current = nullptr;
    // nodeQueue = std::priority_queue<prioritizedNode>();
    // NodeSet openSet, closedSet;
    // nodeMap closedMap;
    // nodeQueue.push(prioritizedNode(new Node(source_)));
    // openSet.insert(new Node(source_));

    auto start = std::chrono::high_resolution_clock::now();

    while (nodeQueue.size()>0) {
    // while (!openSet.empty()) {
        current = nodeQueue.top().qNode;
        // current = *openSet.begin();
        // for (auto node : openSet) {
        //     if (node->getScore() <= current->getScore()) {
        //         current = node;
        //     }
        // }

        if (current->coordinates == target_) {
            break;
        }

        Point pt;
        pt.x = current->coordinates.x
        pt.y = current->coordinates.y
        closedMap.insert(std::make_pair (pt, current->coordinates);
        // closedSet.insert(current);
        // openSet.erase(std::find(openSet.begin(), openSet.end(), current));
        nodeQueue.pop();

        for (uint i = 0; i < directions; ++i) {
            Vec2i newCoordinates(current->coordinates + direction[i]);
            if( inClosedMap(newCoordinates) || rejectWithMap(current->coordinates)) {
              continue;
            }

            // if (detectCollision(newCoordinates) ||
            //     findNodeOnList(closedSet, newCoordinates)) {
            //       if(rejectWithMap(current->coordinates)) {
            //         continue;
            //       }
            // }

            uint totalCost = current->G + ((i < 4) ? 10 : 14);

            Node *successor;
            successor = new Node(newCoordinates, current);
            successor->G = totalCost;
            successor->H = heuristic(successor->coordinates, target_);
            nodeQueue.push(prioritizedNode(successor));

            // ************check if needed************
            // Node *successor = findNodeOnList(openSet, newCoordinates);
            // if (successor == nullptr) {
                // successor = new Node(newCoordinates, current);
                // successor->G = totalCost;
                // successor->H = heuristic(successor->coordinates, target_);
                // openSet.insert(successor);
                // nodeQueue.push(prioritizedNode(successor));
            // }
            // else if (totalCost < successor->G) {
                // successor->parent = current;
                // successor->G = totalCost;
            // }
            //
        }
        auto finish = std::chrono::high_resolution_clock::now();
        if(finish-start > 0.09) {
          Vec2i subPt = nodeQueue.top();
          subOptimalPoint.x =  subPt.x;
          subOptimalPoint.y =  subPt.y;
          return false;
        }
    }
    path.clear();
    return true;

    // CoordinateList path;
    // while (current != nullptr) {
    //     path.push_back(current->coordinates);
    //     current = current->parent;
    // }
    // nodeQueue = std::priority_queue<prioritizedNode>();
    // releaseNodes(openSet);
    // releaseNodes(closedSet);
    // return path;
}

bool AStar::Generator::getPath()
{
  while (current != nullptr) {
    subOptimalPoint.x = current->coordinates.x;
    subOptimalPoint.y = current->coordinates.y;
    path.push_back(current->coordinates);
    current = current->parent;
    updateParentToMap(currentPt, parentPt);
    return false;
  }
  nodeQueue = std::priority_queue<prioritizedNode>();
  releaseNodes(openSet);
  releaseNodes(closedSet);
  return true;
}

AStar::Generator::updateParentToMap(currentPt, parentPt) {
  Point ptCurr, ptPar;
  ptCurr.x = currentPt.x;
  ptCurr.y = currentPt.y;
  ptPar.x = parentPt.x;
  ptPar.y = parentPt.y;
  index = mapObjPtr->find(ptCurr)
  if(index != mapObjPtr->end()) {
    if(index->second.grType() == EMPTY){
      index->second.parent = ptParent;
    }
}

AStar::Node* AStar::Generator::findNodeOnList(NodeSet& nodes_, Vec2i coordinates_)
{
    for (auto node : nodes_) {
        if (node->coordinates == coordinates_) {
            return node;
        }
    }
    return nullptr;
}

bool AStar::Generator::rejectWithMap(Vec2i coordinates_)
{
  Point pt;
  pt.x = coordinates_.x;
  pt.y = coordinates_.y;
  index = mapObjPtr->find(pt)
  if(index != mapObjPtr->end()) {
    if(index->second.grType() == EMPTY){
      return true;
    }
    else {
      return false;
    }
  }
  else {
    return false;
  }
}

bool AStar::Generator::inClosedMap(Vec2i coord)
{
    Point pt;
    pt.x = coord.x;
    pt.y = coord.y;
    index = .find(pt)
    if(index != closedMap->end()) {
      return true;
    }
    else {
      return false;
    }
}

void AStar::Generator::releaseNodes(NodeSet& nodes_)
{
    for (auto it = nodes_.begin(); it != nodes_.end();) {
        delete *it;
        it = nodes_.erase(it);
    }
}

bool AStar::Generator::detectCollision(Vec2i coordinates_)
{
    if (coordinates_.x < 0 || coordinates_.x >= worldSize.x ||
        coordinates_.y < 0 || coordinates_.y >= worldSize.y ||
        std::find(walls.begin(), walls.end(), coordinates_) != walls.end()) {
        return true;
    }
    return false;
}

AStar::Vec2i AStar::Heuristic::getDelta(Vec2i source_, Vec2i target_)
{
    return{ abs(source_.x - target_.x),  abs(source_.y - target_.y) };
}

AStar::uint AStar::Heuristic::manhattan(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * (delta.x + delta.y));
}

AStar::uint AStar::Heuristic::euclidean(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
}

AStar::uint AStar::Heuristic::octagonal(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return 10 * (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y);
}

void AStar::Generator::init(Vec2i src, Vec2i targ, mapObj* ptr) {
  source_ = src;
  target_ = targ;
  mapObjPtr = ptr

  *current = nullptr;
  nodeQueue = std::priority_queue<prioritizedNode>();
  nodeQueue.push(prioritizedNode(new Node(source_)));
}
