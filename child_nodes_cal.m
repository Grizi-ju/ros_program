function child_nodes = child_nodes_cal(parent_node, m, n, obs, closeList)

child_nodes = [];
field = [1,1; n,1; n,m; 1,m];

% 第1个子节点
child_node = [parent_node(1)-1, parent_node(2)+1];
if inpolygon(child_node(1), child_node(2), field(:,1), field(:,2))%判断点是否在多边形内
    if ~ismember(child_node, obs, 'rows')
        child_nodes = [child_nodes; child_node];
    end
end

% 第2个子节点
child_node = [parent_node(1), parent_node(2)+1];
if inpolygon(child_node(1), child_node(2), field(:,1), field(:,2))
    if ~ismember(child_node, obs, 'rows')
        child_nodes = [child_nodes; child_node];
    end
end

% 第3个子节点
child_node = [parent_node(1)+1, parent_node(2)+1];
if inpolygon(child_node(1), child_node(2), field(:,1), field(:,2))
    if ~ismember(child_node, obs, 'rows')
        child_nodes = [child_nodes; child_node];
    end
end

% 第4个子节点
child_node = [parent_node(1)-1, parent_node(2)];
if inpolygon(child_node(1), child_node(2), field(:,1), field(:,2))
    if ~ismember(child_node, obs, 'rows')
        child_nodes = [child_nodes; child_node];
    end
end

% 第5个子节点
child_node = [parent_node(1)+1, parent_node(2)];
if inpolygon(child_node(1), child_node(2), field(:,1), field(:,2))
    if ~ismember(child_node, obs, 'rows')
        child_nodes = [child_nodes; child_node];
    end
end

% 第6个子节点
child_node = [parent_node(1)-1, parent_node(2)-1];
if inpolygon(child_node(1), child_node(2), field(:,1), field(:,2))
    if ~ismember(child_node, obs, 'rows')
        child_nodes = [child_nodes; child_node];
    end
end

% 第7个子节点
child_node = [parent_node(1), parent_node(2)-1];
if inpolygon(child_node(1), child_node(2), field(:,1), field(:,2))
    if ~ismember(child_node, obs, 'rows')
        child_nodes = [child_nodes; child_node];
    end
end

% 第8个子节点
child_node = [parent_node(1)+1, parent_node(2)-1];
if inpolygon(child_node(1), child_node(2), field(:,1), field(:,2))
    if ~ismember(child_node, obs, 'rows')
        child_nodes = [child_nodes; child_node];
    end
end

%% 排除已经存在于closeList的节点
delete_idx = [];
for i = 1:size(child_nodes, 1)
    if ismember(child_nodes(i,:), closeList , 'rows')
        delete_idx(end+1,:) = i;
    end
end
child_nodes(delete_idx, :) = [];
