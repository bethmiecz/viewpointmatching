import GLRenderer as GL

renderer = GL.setup_renderer(GL.CameraIntrinsics(), GL.DepthMode(), gl_version=(4,1))

import Revise
import GLRenderer as GL
import Images as I
import MiniGSG as S
import Rotations as R
import PoseComposition: Pose, IDENTITY_POSE, IDENTITY_ORN
import InverseGraphics as T
import NearestNeighbors
import LightGraphs as LG
import Gen
import Random
try
    import MeshCatViz as V
catch
    import MeshCatViz as V    
end
V.setup_visualizer()
import ImageView as IV
import JSON


YCB_DIR = joinpath(dirname(dirname(pathof(T))),"data")
world_scaling_factor = 10.0
id_to_cloud, id_to_shift, id_to_box  = T.load_ycbv_models_adjusted(YCB_DIR, world_scaling_factor);
all_ids = sort(collect(keys(id_to_cloud)));
names = T.load_ycb_model_list(YCB_DIR)

camera = GL.CameraIntrinsics()
renderer = GL.setup_renderer(camera, GL.DepthMode(); gl_version=(3,3))
obj_paths = T.load_ycb_model_obj_file_paths(YCB_DIR)
for id in all_ids
    mesh = GL.get_mesh_data_from_obj_file(obj_paths[id])
    mesh = T.scale_and_shift_mesh(mesh, world_scaling_factor, id_to_shift[id])
    GL.load_object!(renderer, mesh)
end


function place_objects(config, all_obj_ids, rot)
    if rot == 0
        rotation = [0.7,1.05,0.87,0.5,0.95,0.18,0.2,0.16,0.42,0.15,1.2,1.24,0.27,0.4,0.28,1.03,0.6,0.1,0.2,0.15,0.26, 10.0]
    elseif rot == pi/2
        rotation = [0.5,0.82,0.47,0.33,0.34,0.43,0.63,0.5,0.29,0.9,0.7,0.34,0.8,0.45,0.92,0.5,1.0,0.6,0.85,0.83,0.4, 10.0]
    end
    
    zvals = []
    for z in 1:3
        append!(zvals,rotation[all_obj_ids[z]])
    end
    
    if config == "line"
        xvals = [1.5,0.0,-2.5]
        yvals = [0.0,0.0,0.0]
    elseif config == "nonequtriangle"
        xvals = [-1.5,1.5,1.5]
        yvals = [1.0,-1.5,1.5]
    elseif config == "random"
        xvals = [0.5,0.0,-1.0]
        yvals = [1.75,-1.75,0.75]
    end
    
    all_object_poses = [
        Pose([xvals[1], yvals[1], zvals[1]],R.RotX(rot)),
        Pose([xvals[2], yvals[2], zvals[2]],R.RotX(rot)),
        Pose([xvals[3], yvals[3], zvals[3]],R.RotX(rot))
        ]
    all_clouds = hcat([
        GL.move_points_to_frame_b(id_to_cloud[obj_id], pose)
        for (obj_id, pose) in zip(all_obj_ids, all_object_poses)
    ]...)
    V.reset_visualizer()
    V.viz(all_clouds ./ 10.0)
    return all_object_poses
    end

# +
all_obj_ids = [1,2,3]
config = "nonequtriangle"
all_object_poses = place_objects(config, all_obj_ids, 0)

all_obj_ids2 = [9,12,14]
config2 = "random"
all_object_poses2 = place_objects(config2, all_obj_ids2, 0)
# -

id_to_box[1]

allobjs = [2,4,5,7,10,11,12,13,14,18]
for obj in allobjs
    print(names[obj])
    print(" ")
    print(id_to_box[obj])
    print("\n")
end

# Setting up a renderer for realistic RGB images 
renderer2 = GL.setup_renderer(camera, GL.TextureMixedMode(); gl_version=(3,3))
texture_paths = T.load_ycb_model_texture_file_paths(YCB_DIR)
obj_paths = T.load_ycb_model_obj_file_paths(YCB_DIR)
for id in all_ids
    mesh = GL.get_mesh_data_from_obj_file(obj_paths[id];tex_path=texture_paths[id])
    mesh = T.scale_and_shift_mesh(mesh, world_scaling_factor, id_to_shift[id])
    GL.load_object!(renderer2, mesh)
end

radius = 6.5
z = 6.5
inferred_poses = [
    Pose([sin(ang)*radius, cos(ang)*radius, z], R.RotZ(-ang)*R.RotX(-atan(radius / z))*R.RotY(pi))
    for ang in 0.0:pi/4:2*pi
]

# +
cam_pose = inferred_poses[3]

# table_bbox = S.Box(10.0, 1, 10.0)
# table_mesh = GL.box_mesh_from_dims(T.get_dims(table_bbox))
# GL.load_object!(renderer2, table_mesh)
# table_pose = Pose([0, 0, 10], R.RotX(-90) * R.RotY(cam_pose.orientation[2]))

all_ids = vcat(all_obj_ids, 22)
# all_poses = vcat(all_object_poses, table_pose)

rgb_image,depth_image = GL.gl_render(renderer2, all_obj_ids, [inv(cam_pose)*p for p in all_object_poses], IDENTITY_POSE;
    colors=[I.colorant"tan" for _ in 1:(length(all_obj_ids) + 1)]
)

i = GL.view_image(depth_image)
IV.imshow(i)

# +
# How to get depth images 
depth_images = [
    GL.gl_render(renderer, all_obj_ids, all_object_poses, cam_pose) for cam_pose in inferred_poses
];

depth_images2 = [
    GL.gl_render(renderer, all_obj_ids2, all_object_poses2, cam_pose) for cam_pose in inferred_poses
];
# -

IV.imshow(depth_images[1])
IV.imshow(depth_images2[1])


# obj_id = 10
# depth_images = [
#     GL.gl_render(renderer, [obj_id], [IDENTITY_POSE], cam_pose) for cam_pose in inferred_poses
# ];
clouds = [
    GL.depth_image_to_point_cloud(d, camera) for d in depth_images
];

pose_adjusted_cloud = hcat([T.move_points_to_frame_b(c,p) for (c,p) in zip(clouds, inferred_poses)]...);
V.viz(pose_adjusted_cloud ./ 10.0)

RESOLUTION = 0.07
mins,maxs = T.min_max(pose_adjusted_cloud)
@show mins, maxs
mins,maxs = floor.(Int,mins ./ RESOLUTION) * RESOLUTION , ceil.(Int, maxs ./RESOLUTION) * RESOLUTION
mins .-= RESOLUTION * 5
maxs .+= RESOLUTION * 5

dimensions = [length(collect(mins[1]:RESOLUTION:maxs[1])),
              length(collect(mins[2]:RESOLUTION:maxs[2])),
              length(collect(mins[3]:RESOLUTION:maxs[3]))]
ALL_VOXELS = hcat([[a,b,c] for a in collect(mins[1]:RESOLUTION:maxs[1])
                           for b in collect(mins[2]:RESOLUTION:maxs[2])
                           for c in collect(mins[3]:RESOLUTION:maxs[3])]...);
@show size(ALL_VOXELS)


alphas = zeros(size(ALL_VOXELS)[2])
betas = zeros(size(ALL_VOXELS)[2])
# Inferring occupancy from all views -- want to change this to two views
for t in 1:length(inferred_poses)
    # Occupied, occluded, and free
    occ, occl, free = T.get_occ_ocl_free(T.get_points_in_frame_b(ALL_VOXELS, inferred_poses[t]), camera, depth_images[t], 2*RESOLUTION)
    alphas[occ] .+= 1.0
    betas[free] .+= 1.0
end
p = alphas ./ (alphas .+ betas)
p[isnan.(p)] .= 0.5;


V.reset_visualizer()
V.viz(ALL_VOXELS[:, p.>0.5] ./ 10.0; color=I.colorant"black", channel_name=:occupied)
V.viz(ALL_VOXELS[:, p.==0.5]./ 10.0; color=I.colorant"red", channel_name=:uncertain)

viewpoint1 = T.get_occ_ocl_free(T.get_points_in_frame_b(ALL_VOXELS, inferred_poses[1]), camera, depth_images[1], 2*RESOLUTION)
viewpoint2 = T.get_occ_ocl_free(T.get_points_in_frame_b(ALL_VOXELS, inferred_poses[2]), camera, depth_images[2], 2*RESOLUTION)
viewpoint3 = T.get_occ_ocl_free(T.get_points_in_frame_b(ALL_VOXELS, inferred_poses[2]), camera, depth_images2[1], 2*RESOLUTION)

# allvox = size of ALL_VOXELS
# occupied, occluded, free
function getConsistency(viewpoint1, viewpoint2, epsilon)
    vox_likelihoods = zeros(length(viewpoint1[1]))
    for i in 1:length(viewpoint1[1])
        if ((viewpoint1[1][i] == 1 && viewpoint2[1][i] == 1) || (viewpoint1[3][i] == 1 && viewpoint2[3][i] == 1))
            vox_likelihoods[i] = 1 - epsilon
        elseif ((viewpoint1[1][i] == 1 && viewpoint2[3][i] == 1) ||
            (viewpoint1[3][i] == 1 && viewpoint2[1][i] == 1))
            vox_likelihoods[i] = epsilon
        else
            vox_likelihoods[i] = 0.5
        end
    end
    return vox_likelihoods
end


vox_likelihood = getConsistency(viewpoint1, viewpoint3, 0.001)
V.reset_visualizer()
V.viz(ALL_VOXELS[:, vox_likelihood.<0.5] ./ 10.0; color=I.colorant"black", channel_name=:occupied)
V.viz(ALL_VOXELS[:, vox_likelihood.==0.5]./ 10.0; color=I.colorant"red", channel_name=:uncertain)

allobjs = [2,4,5,7,10,11,12,13,14,18]
permutations = []
for obj1 in allobjs
    for obj2 in allobjs
        for obj3 in allobjs
            if (obj1 != obj2) && (obj1 != obj3) && (obj2 != obj3)
                push!(permutations,[obj1,obj2,obj3])
            end
        end
    end
end

print(length(permutations))
print(permutations[1:10])

# +
# TODO in future: rotate objects differently and match scenes with same objects but different rotations
# Render and save all scenes
# table_bbox = S.Box(10.0, 1.0, 10.0)
# table_mesh = GL.box_mesh_from_dims(T.get_dims(table_bbox))
# GL.load_object!(renderer2, table_mesh)

# Saving RGB images
configs = ["line", "nonequtriangle", "random"]
for objs in permutations
    for config in configs
        count = 1
        for cam_pose in inferred_poses
            # Set table pose for scene
#             table_pose = Pose([0, 0, 10], R.RotX(-90) * R.RotY(cam_pose.orientation[2]))
            # Place three objects and generate images
            all_object_poses = place_objects(config, objs, 0)
            rgb_image,depth_image = GL.gl_render(renderer2, objs, [inv(cam_pose)*p for p in all_object_poses], IDENTITY_POSE;
                colors=[I.colorant"tan" for _ in 1:(length(all_obj_ids) + 1)] 
            )
            # Make directories and image names
            currdir = pwd() * "/scenes/" 
            newdir = string(objs[1]) * "_" * string(objs[2]) * "_" * string(objs[3]) 
            if !isdir(currdir * newdir)
                mkdir(currdir * newdir)
            end
            
            newdir = newdir * "/" * config
            if !isdir(currdir * newdir)
                mkdir(currdir * newdir)
            end
            
            imname = string(count) * ".png"
            path = currdir * newdir * "/" * imname
            # Save images
            i = GL.view_rgb_image(rgb_image)
            T.FileIO.save(path, i)
            count += 1
        end
    end
end


# +
# Take objects from JSON file and copy to diff directory
# open("3dp3behavior.json","r") do f
#     global trials = JSON.parse(f)
# end

open("practice.json","r") do f
    global trials = JSON.parse(f)
end

function extractImgFromJSON(filename)
    str = split(filename, '/')
    trip = str[3]
    config = str[4]
    view = str[5]
    return trip,config,view
end

function makeDirs(filename)
    trip,config,view = extractImgFromJSON(filename)
    if !(isdir(currdir * "/" * trip))
        mkdir(currdir * "/" * trip)
    end
    if !(isdir(currdir * "/" * trip * "/" * config))
        mkdir(currdir * "/" * trip * "/" * config)
    end
    if !isfile(currdir * "/" * trip * "/" * config * "/" * view)
        cp(pastdir * "/" * trip * "/" * config * "/" * view, currdir * "/" * trip * "/" * config * "/" * view)
    end
end

currdir = "/Users/bethmieczkowski/Desktop/InverseGraphics/notebooks/trialscenes"
pastdir = "/Users/bethmieczkowski/Desktop/InverseGraphics/notebooks/scenes"
for trial in trials
    s = trial["img_sample"]
    l = trial["img_lure"]
    t = trial["img_target"]
    makeDirs(s)
    makeDirs(l)
    makeDirs(t)
end
# -

print(length(trials))

# +
objset = []
for obj1 in allobjs
    for obj2 in allobjs
        for obj3 in allobjs
            if (obj1 != obj2) && (obj1 != obj3) && (obj2 != obj3)
                s = Set([obj1,obj2,obj3])
                for config in configs
                    t = [s,config]
                    if !(t in objset)
                        push!(objset,t)
                    end
                end
            end
        end
    end
end

# Make JSON file for behavioral experiment
valid = true
currdir = "./scenes/" 
allfiles = []
count = 1
for x in 1:4
    for _ in 1:30
        # lure and target need same view 
        possibleviews = Random.shuffle([1,2,3,4,5,6,7,8,9])
        view1 = possibleviews[1]
        view2 = possibleviews[2]
        view3 = rand(1:9)
        if x == 1
            randidx = rand(1:length(objset))
            objconfig = objset[randidx]
            randplace1 = Random.shuffle(collect(objconfig[1]))
            while true
                global randplace2 = Random.shuffle(collect(objconfig[1]))
                if randplace1 != randplace2
                    break
                end
            end
            config = objconfig[2]
            global m = currdir * string(randplace1[1]) * "_" * string(randplace1[2]) * "_" * string(randplace1[3]) * "/" * config * "/" * string(view1) * ".png"
            global target = currdir * string(randplace1[1]) * "_" * string(randplace1[2]) * "_" * string(randplace1[3]) * "/" * config * "/" * string(view2) * ".png"
            global lure = currdir * string(randplace2[1]) * "_" * string(randplace2[2]) * "_" * string(randplace2[3]) * "/" * config * "/" * string(view3) * ".png"
            deleteat!(objset, randidx)
        end
        if x == 2
            randidx = rand(1:length(objset))
            objconfig = objset[randidx]
            randplace1 = Random.shuffle(collect(objconfig[1]))
            randplace2 = Random.shuffle(collect(objconfig[1]))   
            altconfigs = [c for c in configs if c != objconfig[2]]
            config1 = objconfig[2]
            config2 = Random.shuffle(altconfigs)[1]
            global m = currdir * string(randplace1[1]) * "_" * string(randplace1[2]) * "_" * string(randplace1[3]) * "/" * config1 * "/" * string(view1) * ".png"
            global target = currdir * string(randplace1[1]) * "_" * string(randplace1[2]) * "_" * string(randplace1[3]) * "/" * config1 * "/" * string(view2) * ".png"
            global lure = currdir * string(randplace2[1]) * "_" * string(randplace2[2]) * "_" * string(randplace2[3]) * "/" * config2 * "/" * string(view3) * ".png"
            deleteat!(objset, randidx)
            j = findall(x->x==[objconfig[1],config2], objset)
            deleteat!(objset, j)
        end
        if x == 3
            config = Random.shuffle(configs)[1]
            while true
                randidx1 = rand(1:length(objset))
                global objconfig1 = objset[randidx1]
                if [objconfig1[1],config] in objset
                    break
                end
            end
            i = findall(x->x==[objconfig1[1],config], objset)
            deleteat!(objset, i)
            randplace1 = Random.shuffle(collect(objconfig1[1]))
            while true
                randidx2 = rand(1:length(objset))
                global objconfig2 = objset[randidx2]
                if [objconfig2[1],config] in objset
                    break
                end
            end
            randplace2 = Random.shuffle(collect(objconfig2[1]))
            global m = currdir * string(randplace1[1]) * "_" * string(randplace1[2]) * "_" * string(randplace1[3]) * "/" * config * "/" * string(view1) * ".png"
            global target = currdir * string(randplace1[1]) * "_" * string(randplace1[2]) * "_" * string(randplace1[3]) * "/" * config * "/" * string(view2) * ".png"
            global lure = currdir * string(randplace2[1]) * "_" * string(randplace2[2]) * "_" * string(randplace2[3]) * "/" * config * "/" * string(view3) * ".png"
            j = findall(x->x==[objconfig2[1],config], objset)
            deleteat!(objset, j)
        end
        if x == 4
            config1 = Random.shuffle(configs)[1]
            while true
                randidx1 = rand(1:length(objset))
                global objconfig1 = objset[randidx1]
                if [objconfig1[1],config1] in objset
                    break
                end
            end
            i = findall(x->x==[objconfig1[1],config1], objset)
            deleteat!(objset, i)
            randplace1 = Random.shuffle(collect(objconfig1[1]))
            altconfigs = [c for c in configs if c != config1]
            config2 = Random.shuffle(altconfigs)[1]
            while true
                randidx2 = rand(1:length(objset))
                global objconfig2 = objset[randidx2]
                if [objconfig2[1],config2] in objset
                    break
                end
            end
            randplace2 = Random.shuffle(collect(objconfig2[1]))
            global m = currdir * string(randplace1[1]) * "_" * string(randplace1[2]) * "_" * string(randplace1[3]) * "/" * config1 * "/" * string(view1) * ".png"
            global target = currdir * string(randplace1[1]) * "_" * string(randplace1[2]) * "_" * string(randplace1[3]) * "/" * config1 * "/" * string(view2) * ".png"
            global lure = currdir * string(randplace2[1]) * "_" * string(randplace2[2]) * "_" * string(randplace2[3]) * "/" * config2 * "/" * string(view3) * ".png"
            j = findall(x->x==[objconfig2[1],config2], objset)
            deleteat!(objset, j)
        end
        files = Dict{String,String}()
        files["img_sample"] = m
        files["img_target"] = target
        files["img_lure"] = lure
        files["lure_type"] = string(x)
        files["trialid"] = string(count)
        push!(allfiles,files)
        count += 1
    end
end
# -


# print(length(allfiles))
print(allfiles[1])

# shuffledfiles = Random.shuffle(allfiles)
open("3dp3behavior.json","w") do f
    JSON.print(f, allfiles)
end

# Create practice trials JSON from remaining trials 
practice = []
for _ in 1:5
    possibleviews = Random.shuffle([1,2,3,4,5,6,7,8,9])
    view1 = possibleviews[1]
    view2 = possibleviews[2]
    view3 = rand(1:9)
    config1 = Random.shuffle(configs)[1]
    while true
        randidx1 = rand(1:length(objset))
        global objconfig1 = objset[randidx1]
        if [objconfig1[1],config1] in objset
            break
        end
    end
    i = findall(x->x==[objconfig1[1],config1], objset)
    deleteat!(objset, i)
    randplace1 = Random.shuffle(collect(objconfig1[1]))
    altconfigs = [c for c in configs if c != config1]
    config2 = Random.shuffle(altconfigs)[1]
    while true
        randidx2 = rand(1:length(objset))
        global objconfig2 = objset[randidx2]
        if [objconfig2[1],config2] in objset
            break
        end
    end
    randplace2 = Random.shuffle(collect(objconfig2[1]))
    global m = currdir * string(randplace1[1]) * "_" * string(randplace1[2]) * "_" * string(randplace1[3]) * "/" * config1 * "/" * string(view1) * ".png"
    global target = currdir * string(randplace1[1]) * "_" * string(randplace1[2]) * "_" * string(randplace1[3]) * "/" * config1 * "/" * string(view2) * ".png"
    global lure = currdir * string(randplace2[1]) * "_" * string(randplace2[2]) * "_" * string(randplace2[3]) * "/" * config2 * "/" * string(view3) * ".png"
    j = findall(x->x==[objconfig2[1],config2], objset)
    deleteat!(objset, j)
    
    files = Dict{String,String}()
    files["img_sample"] = m
    files["img_target"] = target
    files["img_lure"] = lure
    files["lure_type"] = "practice"
    files["trialid"] = string(count)
    push!(practice,files)
    count += 1
end

open("practice.json","w") do f
    JSON.print(f, practice)
end

# Take in JSON files and get model match for images
open("3dp3behavior.json","r") do f
    global trials = JSON.parse(f)
end

camera = GL.CameraIntrinsics()
renderer = GL.setup_renderer(camera, GL.DepthMode(); gl_version=(3,3))
obj_paths = T.load_ycb_model_obj_file_paths(YCB_DIR)
for id in all_ids
    mesh = GL.get_mesh_data_from_obj_file(obj_paths[id])
    mesh = T.scale_and_shift_mesh(mesh, world_scaling_factor, id_to_shift[id])
    GL.load_object!(renderer, mesh)
end

# +
function extractImgFromJSON(filename)
    str = split(filename, '/')
    trip = split(str[3], '_')
    trip = [parse(Int, ss) for ss in trip]
    config = str[4]
    view = split(str[5], '.')[1]
    view = [parse(Int, ss) for ss in view]
    return trip,config,view
end

function getVpt(trip,config,view)
    cam_pose = inferred_poses[view][1]
    poses = place_objects(config, trip, 0)
    
    depth_img = GL.gl_render(renderer, trip, poses, cam_pose)
    vpt = T.get_occ_ocl_free(T.get_points_in_frame_b(ALL_VOXELS, cam_pose), camera, depth_img, 2*RESOLUTION)
    return vpt
end

function getMatch(s,l,t)
    s_trip,s_config,s_view = extractImgFromJSON(s)
    l_trip,l_config,l_view = extractImgFromJSON(l)
    t_trip,t_config,t_view = extractImgFromJSON(t)
    
    s_vpt = getVpt(s_trip,s_config,s_view)
    l_vpt = getVpt(l_trip,l_config,l_view)
    t_vpt = getVpt(t_trip,t_config,t_view)
    
#     vox_likelihood_sl = getConsistency(s_vpt, l_vpt, 0.001)
#     vox_likelihood_st = getConsistency(s_vpt, t_vpt, 0.001)
#     print(vox_likelihood_sl)
#     return vox_likelihood_sl, vox_likelihood_st
end

for trial in trials[1:2]
    s = trial["img_sample"]
    l = trial["img_lure"]
    t = trial["img_target"]
    ltype = trial["lure_type"]

    vox_likelihood_sl,vox_likelihood_st = getMatch(s,l,t)
    print("\n")
end
# -


