/**
* \author Xuan Zhang
* \data Mai-July 2018
*/
namespace octomap {

// Tree implementation  --------------------------------------
template <class SEMANTICS>
LocalSemanticsOcTree<SEMANTICS>::LocalSemanticsOcTree(double resolution)
    : OccupancyOcTreeBase<LocalSemanticsOcTreeNode<SEMANTICS>>(this->resolution)
{
    semanticsOcTreeMemberInit.ensureLinking();
};

template <class SEMANTICS>
bool LocalSemanticsOcTree<SEMANTICS>::pruneNode(LocalSemanticsOcTreeNode<SEMANTICS>* node)
{
    // Same as ColorOcTree
    if (!isNodeCollapsible(node))
        return false;

    // Set value to children's values (all assumed equal)
    node->copyData(*(this->getNodeChild(node, 0)));

    if (node->isColorSet()) // TODO check
        node->setColor(node->getAverageChildColor());

    // Delete children
    for (unsigned int i = 0; i < 8; i++) {
        this->deleteNodeChild(node, i);
    }
    delete[] node->children;
    node->children = NULL;

    return true;
}

template <class SEMANTICS>
bool LocalSemanticsOcTree<SEMANTICS>::isNodeCollapsible(const LocalSemanticsOcTreeNode<SEMANTICS>* node) const
{
    // All children must exist, must not have children of
    // their own and have same occupancy probability and same semantic
    if (!this->nodeChildExists(node, 0))
        return false;
    const LocalSemanticsOcTreeNode<SEMANTICS>* firstChild = this->getNodeChild(node, 0);
    if (this->nodeHasChildren(firstChild))
        return false;
    for (unsigned int i = 1; i < 8; i++) {
        // Compare nodes using their occupancy and semantics, ignoring color for pruning
        if (!this->nodeChildExists(node, i) || this->nodeHasChildren(this->getNodeChild(node, i))
            || !(this->getNodeChild(node, i)->getValue() == firstChild->getValue())
            || !(this->getNodeChild(node, i)->getSemantics() == firstChild->getSemantics()))
            return false;
    }
    return true;
}

template <class SEMANTICS>
void LocalSemanticsOcTree<SEMANTICS>::setUseSemanticColor(bool use)
{
    // Traverse all tree nodes
    for (typename LocalSemanticsOcTree<SEMANTICS>::tree_iterator it = this->begin_tree(), end = this->end_tree(); it != end; ++it)
        it->use_semantic_color = use;
}

template <class SEMANTICS>
LocalSemanticsOcTreeNode<SEMANTICS>* LocalSemanticsOcTree<SEMANTICS>::setNodeColor(const OcTreeKey& key,
    uint8_t r,
    uint8_t g,
    uint8_t b)
{
    LocalSemanticsOcTreeNode<SEMANTICS>* n = this->search(key);
    if (n != 0) {
        n->setColor(r, g, b);
    }
    return n;
}

template <class SEMANTICS>
LocalSemanticsOcTreeNode<SEMANTICS>* LocalSemanticsOcTree<SEMANTICS>::averageNodeColor(const OcTreeKey& key, uint8_t r,
    uint8_t g, uint8_t b)
{
    LocalSemanticsOcTreeNode<SEMANTICS>* n = this->search(key);
    if (n != 0) {
        if (n->isColorSet()) {
            ColorOcTreeNode::Color prev_color = n->getColor();
            n->setColor((prev_color.r + r) / 2, (prev_color.g + g) / 2, (prev_color.b + b) / 2);
        } else {
            n->setColor(r, g, b);
        }
    }
    return n;
}

template <class SEMANTICS>
LocalSemanticsOcTreeNode<SEMANTICS>* LocalSemanticsOcTree<SEMANTICS>::updateNodeSemantics(const OcTreeKey& key, SEMANTICS obs)
{
    LocalSemanticsOcTreeNode<SEMANTICS>* n = this->search(key);
    if (n != 0) {
        if (n->isSemanticsSet()) {
            SEMANTICS sem = SEMANTICS::semanticFusion(n->semantics, obs);
            n->setSemantics(sem);
        } else {
            n->setSemantics(obs);
        }
    }
    return n;
}

template <class SEMANTICS>
void LocalSemanticsOcTree<SEMANTICS>::updateInnerOccupancy()
{
    this->updateInnerOccupancyRecurs(this->root, 0);
}

template <class SEMANTICS>
void LocalSemanticsOcTree<SEMANTICS>::updateInnerOccupancyRecurs(LocalSemanticsOcTreeNode<SEMANTICS>* node, unsigned int depth)
{
    // Only recurse and update for inner nodes:
    if (this->nodeHasChildren(node)) {
        // Return early for last level:
        if (depth < this->tree_depth) {
            for (unsigned int i = 0; i < 8; i++) {
                if (this->nodeChildExists(node, i)) {
                    updateInnerOccupancyRecurs(this->getNodeChild(node, i), depth + 1);
                }
            }
        }
        // Update occupancy, color and semantics for inner nodes
        node->updateOccupancyChildren();
        node->updateColorChildren();
        node->updateSemanticsChildren();
    }
}
#if 1
template <class SEMANTICS>
unsigned int LocalSemanticsOcTree<SEMANTICS>::getLastUpdateTime()
{
    // this value is updated whenever inner nodes are
    // updated using updateOccupancyChildren()
    return LocalSemanticsOcTree<SEMANTICS>::root->getTimestamp();
}

// 删除过期的节点
template <class SEMANTICS>
void LocalSemanticsOcTree<SEMANTICS>::degradeOutdatedNodes(unsigned int time_thres)
{
    // 当前查询时间
    unsigned int query_time = (unsigned int)time(NULL);

    // 遍历每个叶节点
    for (typename LocalSemanticsOcTree<SEMANTICS>::leaf_iterator it = this->begin_leafs(), end = this->end_leafs(); it != end; ++it) {
        // 如果节点是占用的，并且内部时间超过阈值就删除
        // 或者使用空间欧式距离作为判断阈值，计算所有走过节点的 (x, y, z) 与当前位置的距离，超过指定阈值不显示
    #if 1
        if (this->isNodeOccupied(*it) && ((query_time - it->getTimestamp()) > time_thres)) {
          //integrateMissNoTime(&*it);
          this->deleteNode(it.getKey());
        }
    #else
        if (this->isNodeOccupied(*it) && ((query_time > it->getTimestamp()))) {
            integrateMissNoTime(&*it);
        }
    #endif
    }
}

template <class SEMANTICS>
void LocalSemanticsOcTree<SEMANTICS>::updateNodeLogOdds(LocalSemanticsOcTreeNode<SEMANTICS>* node, const float& update) const {
    OccupancyOcTreeBase<LocalSemanticsOcTreeNode<SEMANTICS>>::updateNodeLogOdds(node, update);
    node->updateTimestamp();
}

template <class SEMANTICS>
void LocalSemanticsOcTree<SEMANTICS>::integrateMissNoTime(LocalSemanticsOcTreeNode<SEMANTICS>* node) const {
    OccupancyOcTreeBase<LocalSemanticsOcTreeNode<SEMANTICS>>::updateNodeLogOdds(node, LocalSemanticsOcTree<SEMANTICS>::prob_miss_log);
}

#endif

template <class SEMANTICS>
typename LocalSemanticsOcTree<SEMANTICS>::StaticMemberInitializer LocalSemanticsOcTree<SEMANTICS>::semanticsOcTreeMemberInit;

} // end namespace
