
algs_to_test_dict = {

        ################################################################################################################
        ################################################################################################################
        ################################################################################################################

        '5-5-SDS': (run_k_sds, {   # for random and empty - 10, for warehouse 30, for game 2: k=h=15
            'k': 5,
            'h': 5,
            'p_ch_type': 'max_prev',
            # 'pref_paths_type': 'pref_index',
            'pref_paths_type': 'pref_path_length',
            'p_h': 0.9,
            'p_l': 0.9,
            # 'limit_type': 'norm_time',
            'limit_type': 'dist_time',
            # 'limit_type': 'dist_a_star_closed',
            'dist': True,
            'color': 'red',
        }),

        '15-5-SDS': (run_k_sds, {  # for random and empty - 10, for warehouse 30, for game 2: k=h=15
            'k': 15,
            'h': 5,
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

        '30-5-SDS': (run_k_sds, {  # for random and empty - 10, for warehouse 30, for game 2: k=h=15
            'k': 30,
            'h': 5,
            'p_ch_type': 'max_prev',
            # 'pref_paths_type': 'pref_index',
            'pref_paths_type': 'pref_path_length',
            'p_h': 0.9,
            'p_l': 0.9,
            # 'limit_type': 'norm_time',
            'limit_type': 'dist_time',
            # 'limit_type': 'dist_a_star_closed',
            'dist': True,
            'color': 'green',
        }),

        '15-15-SDS': (run_k_sds, {  # for random and empty - 10, for warehouse 30, for game 2: k=h=15
            'k': 15,
            'h': 15,
            'p_ch_type': 'max_prev',
            # 'pref_paths_type': 'pref_index',
            'pref_paths_type': 'pref_path_length',
            'p_h': 0.9,
            'p_l': 0.9,
            # 'limit_type': 'norm_time',
            'limit_type': 'dist_time',
            # 'limit_type': 'dist_a_star_closed',
            'dist': True,
            'color': 'c',
        }),

        '30-15-SDS': (run_k_sds, {  # for random and empty - 10, for warehouse 30, for game 2: k=h=15
            'k': 30,
            'h': 15,
            'p_ch_type': 'max_prev',
            # 'pref_paths_type': 'pref_index',
            'pref_paths_type': 'pref_path_length',
            'p_h': 0.9,
            'p_l': 0.9,
            # 'limit_type': 'norm_time',
            'limit_type': 'dist_time',
            # 'limit_type': 'dist_a_star_closed',
            'dist': True,
            'color': 'blue',
        }),

        '30-30-SDS': (run_k_sds, {  # for random and empty - 10, for warehouse 30, for game 2: k=h=15
            'k': 30,
            'h': 30,
            'p_ch_type': 'max_prev',
            # 'pref_paths_type': 'pref_index',
            'pref_paths_type': 'pref_path_length',
            'p_h': 0.9,
            'p_l': 0.9,
            # 'limit_type': 'norm_time',
            'limit_type': 'dist_time',
            # 'limit_type': 'dist_a_star_closed',
            'dist': True,
            'color': 'purple',
        }),

    }