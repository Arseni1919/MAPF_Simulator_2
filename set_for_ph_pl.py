
algs_to_test_dict = {

        ################################################################################################################
        ################################################################################################################
        ################################################################################################################

        'k-SDS-1-1': (run_k_sds, {   # for random and empty - 10, for warehouse 30, for game 2: k=h=15
            'k': 10,
            'h': 10,
            'p_ch_type': 'max_prev',
            # 'pref_paths_type': 'pref_index',
            'pref_paths_type': 'pref_path_length',
            'p_h': 1,
            'p_l': 1,
            # 'limit_type': 'norm_time',
            'limit_type': 'dist_time',
            # 'limit_type': 'dist_a_star_closed',
            'dist': True,
            'color': 'red',
        }),

        'k-SDS-0.9-0.9': (run_k_sds, {  # for random and empty - 10, for warehouse 30, for game 2: k=h=15
            'k': 10,
            'h': 10,
            'p_ch_type': 'max_prev',
            # 'pref_paths_type': 'pref_index',
            'pref_paths_type': 'pref_path_length',
            'p_h': 0.9,
            'p_l': 0.9,
            # 'limit_type': 'norm_time',
            'limit_type': 'dist_time',
            # 'limit_type': 'dist_a_star_closed',
            'dist': True,
            'color': 'orange',
        }),

        'k-SDS-0.1-0.9': (run_k_sds, {  # for random and empty - 10, for warehouse 30, for game 2: k=h=15
            'k': 10,
            'h': 10,
            'p_ch_type': 'max_prev',
            # 'pref_paths_type': 'pref_index',
            'pref_paths_type': 'pref_path_length',
            'p_h': 0.9,
            'p_l': 0.1,
            # 'limit_type': 'norm_time',
            'limit_type': 'dist_time',
            # 'limit_type': 'dist_a_star_closed',
            'dist': True,
            'color': 'green',
        }),

        'k-SDS-0.9-0.1': (run_k_sds, {  # for random and empty - 10, for warehouse 30, for game 2: k=h=15
            'k': 10,
            'h': 10,
            'p_ch_type': 'max_prev',
            # 'pref_paths_type': 'pref_index',
            'pref_paths_type': 'pref_path_length',
            'p_h': 0.1,
            'p_l': 0.9,
            # 'limit_type': 'norm_time',
            'limit_type': 'dist_time',
            # 'limit_type': 'dist_a_star_closed',
            'dist': True,
            'color': 'c',
        }),

        'k-SDS-0.5-0.5': (run_k_sds, {  # for random and empty - 10, for warehouse 30, for game 2: k=h=15
            'k': 10,
            'h': 10,
            'p_ch_type': 'max_prev',
            # 'pref_paths_type': 'pref_index',
            'pref_paths_type': 'pref_path_length',
            'p_h': 0.5,
            'p_l': 0.5,
            # 'limit_type': 'norm_time',
            'limit_type': 'dist_time',
            # 'limit_type': 'dist_a_star_closed',
            'dist': True,
            'color': 'blue',
        }),

        'k-SDS-0.1-0.1': (run_k_sds, {  # for random and empty - 10, for warehouse 30, for game 2: k=h=15
            'k': 10,
            'h': 10,
            'p_ch_type': 'max_prev',
            # 'pref_paths_type': 'pref_index',
            'pref_paths_type': 'pref_path_length',
            'p_h': 0.1,
            'p_l': 0.1,
            # 'limit_type': 'norm_time',
            'limit_type': 'dist_time',
            # 'limit_type': 'dist_a_star_closed',
            'dist': True,
            'color': 'purple',
        }),

    }