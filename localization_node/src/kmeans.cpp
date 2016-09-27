#include "kmeans.h"

kMeans::kMeans(vector<Point> &population, int kClusters, int maxClusterDisparity)
{
    pop = population;
    if(kClusters>0)K = kClusters; else K = 1;
    maxCDisp = maxClusterDisparity;
    maxIterations = maxClusterDisparity;
}

vector<Point> kMeans::getClusters()
{
    int sampleCentroid = 0;
    unsigned int nSamples = pop.size();
    vector<Point> centroids; centroids.clear();
    if(pop.size()==0) return centroids;
    vector<cluster> clusters;
    clusters.resize(K);centroids.clear();
    vector<int>chosenCentroids;
    bool centroidAlreadyExists = false, faraway = false, leaving = true;
    unsigned int matchingTries = 0;
    for(unsigned int k=0;k<K;k++){ // Assign random starting clusterheads
        matchingTries = 0;
        do{
            sampleCentroid = rand()%nSamples;
            centroidAlreadyExists = false;
            faraway = true;
            for(unsigned int centroid = 0; centroid<chosenCentroids.size(); centroid++){
                if(sampleCentroid == chosenCentroids[centroid]) {
                    centroidAlreadyExists = true;
                    break;
                }
                if(squaredDist(pop[sampleCentroid],pop[chosenCentroids[centroid]])>2000) faraway = true;
                else {
                    faraway = false;
                    break;
                }
            }
            matchingTries++;
            if(matchingTries>=nSamples*2){
                leaving = false;
                K = chosenCentroids.size();
                clusters.pop_back();
                break;
            }
        }while(centroidAlreadyExists || !faraway);
        if(leaving){
            chosenCentroids.push_back(sampleCentroid);
            centroids.push_back(pop[sampleCentroid]);
            clusters[k].center = centroids[k];
        }
    }

    for(unsigned int cicle=0;cicle<maxIterations;cicle++){
        // Clear previous clusters
        for(unsigned int k = 0; k < K; k ++){
            clusters[k].members.clear();
        }
         // Assign samples to clusters
        for(unsigned int samp = 0; samp < nSamples; samp++){
            int nearest = nearestCluster(pop[samp],clusters);
            clusters[nearest].members.push_back(pop[samp]);
        }

        // Clear previous clusters
        for(unsigned int k = 0; k < K; k ++){
            clusters[k].center = Point(0,0);
            for(unsigned int mem=0; mem<clusters[k].members.size(); mem++){

                clusters[k].center.x += clusters[k].members[mem].x;
                clusters[k].center.y += clusters[k].members[mem].y;
            }
            if(clusters[k].members.size()>0)clusters[k].center.x /= clusters[k].members.size();
            if(clusters[k].members.size()>0)clusters[k].center.y /= clusters[k].members.size();
        }
    }

    centroids.clear();
    for(unsigned int k = 0; k < K; k ++){
        centroids.push_back(clusters[k].center);
    }
    return centroids;
}

int kMeans::nearestCluster(Point point, vector<cluster> &clusters)
{
    int minDist = 1e9;
    int nearestClust = 0;
    int dist = 0;
    for(unsigned int cluster=0;cluster<clusters.size();cluster++){
        dist = squaredDist(point,clusters[cluster].center);
        if(dist<minDist){
            minDist = dist;
            nearestClust = cluster;
        }
    }

    return nearestClust;
}

int kMeans::squaredDist(Point p1, Point p2)
{
    int dx = 0, dy = 0;
    dx = p1.x - p2.x;
    dy = p1.y - p2.y;
    return dx*dx + dy*dy;
}
