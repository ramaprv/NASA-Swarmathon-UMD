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
    setDiagonalMovement(true);
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
        current = nodeQueue.top().qNode;
        if (current->coordinates == target_) {
            break;
        }

        Point pt;
        pt.x = current->coordinates.x;
        pt.y = current->coordinates.y;
        closedMap.insert(std::make_pair (pt, current->coordinates));
        nodeQueue.pop();

        for (uint i = 0; i < directions; ++i) {
            Vec2i newCoordinates(current->coordinates + direction[i]);
            if( inClosedMap(newCoordinates) || rejectWithMap(current->coordinates)) {
              continue;
            }

            uint totalCost = current->G + ((i < 4) ? 10 : 14);

            Node *successor;
            successor = new Node(newCoordinates, current);
            successor->G = totalCost;
            successor->H = heuristic(successor->coordinates, target_);
            nodeQueue.push(prioritizedNode(successor));

        }
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = finish - start;
        std::chrono::duration<double> threshold = (std::chrono::duration<double>)0.09;
        if(elapsed > threshold) {
          Vec2i subPt = nodeQueue.top().qNode->coordinates;
          subOptimalPoint.x =  subPt.x;
          subOptimalPoint.y =  subPt.y;
          return false;
        }
      }
    path.clear();
    return true;
}

bool AStar::Generator::getPath()
{
  while (current != nullptr) {
    subOptimalPoint.x = current->coordinates.x;
    subOptimalPoint.y = current->coordinates.y;
    Vec2i currentPt, parentPt;
    currentPt = current->coordinates;
    path.push_back(current->coordinates);
    current = current->parent;
    parentPt = current->coordinates;
    updateParentToMap(currentPt, parentPt);
    return false;
  }
  nodeQueue = std::priority_queue<prioritizedNode>();

  return true;
}

void AStar::Generator::updateParentToMap(Vec2i currentPt, Vec2i parentPt) {
  Point ptCurr, ptPar;
  ptCurr.x = currentPt.x;
  ptCurr.y = currentPt.y;
  ptPar.x = parentPt.x;
  ptPar.y = parentPt.y;
  auto index = mapObjPtr->find(ptCurr);
  if(index != mapObjPtr->end()) {
    if(index->second.grType == EMPTY){
      index->second.parent = ptPar;
      index->second.isOptimal = true;
    }
  }
}

bool AStar::Generator::rejectWithMap(Vec2i coordinates_)
{
  Point pt;
  pt.x = coordinates_.x;
  pt.y = coordinates_.y;
  auto index = mapObjPtr->find(pt);
  if(index != mapObjPtr->end()) {
    if(index->second.grType == EMPTY){
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
    auto index = closedMap.find(pt);
    if(index != closedMap.end()) {
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

void AStar::Generator::init(AStar::Vec2i src, Vec2i targ, std::map<Point, mapValue>* ptr) {
  source_ = src;
  target_ = targ;
  mapObjPtr = ptr;

  current = nullptr;
  closedMap.clear();
  nodeQueue = std::priority_queue<prioritizedNode>();
  nodeQueue.push(prioritizedNode(new Node(source_)));
}
