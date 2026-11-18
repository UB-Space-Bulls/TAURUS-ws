"""Derive a real-terrain profile from the reviewed DEIMOS Nav2 configuration."""
from copy import deepcopy


def physical_profile(original, use_sim_time=False):
    data = deepcopy(original)
    if not isinstance(data, dict):
        raise ValueError('Expected a Nav2 YAML mapping')
    for name in ('local_costmap', 'global_costmap'):
        try:
            params = data[name][name]['ros__parameters']
            plugins = params['plugins']
        except (KeyError, TypeError):
            raise ValueError(f'Missing standard {name}/{name}/ros__parameters/plugins') from None
        supported = {'terrain_layer', 'obstacle_layer', 'inflation_layer'}
        if not isinstance(plugins, list) or not set(plugins) <= supported:
            raise ValueError(f'{name}: custom layers require a manual merge; source file untouched')
        if 'obstacle_layer' not in plugins or 'inflation_layer' not in plugins:
            raise ValueError(f'{name}: expected DEIMOS obstacle and inflation layers')
        if params['obstacle_layer'].get('plugin') != 'nav2_costmap_2d::ObstacleLayer':
            raise ValueError(f'{name}: unsupported obstacle plugin')
        if 'terrain_layer' in params and params['terrain_layer'].get('plugin') != 'nav2_costmap_2d::StaticLayer':
            raise ValueError(f'{name}: terrain_layer name belongs to a different plugin')
        if params.get('rolling_window') is not True:
            raise ValueError(f'{name}: this profile expects the DEIMOS rolling window')
        if params.get('filters'):
            raise ValueError(f'{name}: existing costmap filters require a manual review')
        params['plugins'] = ['terrain_layer', 'obstacle_layer', 'inflation_layer']
        params.update(track_unknown_space=True, unknown_cost_value=255,
                      trinary_costmap=False, use_maximum=False, lethal_cost_threshold=100)
        params['terrain_layer'] = {
            'plugin': 'nav2_costmap_2d::StaticLayer', 'enabled': True,
            'map_topic': '/terrain_cost_grid', 'map_subscribe_transient_local': True,
            'subscribe_to_updates': False, 'footprint_clearing_enabled': False,
        }
        # A scan ray says the volume is empty; it does not establish safe ground.
        params['obstacle_layer']['combination_method'] = 2
        params['obstacle_layer']['footprint_clearing_enabled'] = False
    try:
        planner = data['planner_server']['ros__parameters']['GridBased']
        if planner['plugin'] != 'nav2_navfn_planner::NavfnPlanner':
            raise ValueError('Expected the reviewed DEIMOS NavfnPlanner; review unknown policy manually')
        planner['allow_unknown'] = False
    except (KeyError, TypeError):
        raise ValueError('Expected planner_server/GridBased Navfn configuration') from None

    def update_time(node):
        if isinstance(node, dict):
            for key, value in node.items():
                if key == 'ros__parameters' and isinstance(value, dict):
                    value['use_sim_time'] = bool(use_sim_time)
                update_time(value)
    update_time(data)
    return data
