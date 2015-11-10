#include <iostream>
#include <vector>
#include <utility>

using namespace std;

int solve(vector<pair<int, int> >& edges, vector<int> nodes, int sp = 0) {
    if (nodes.size() == 14) {
        return 1;
    }
    int sum = 0;
    for (int i = sp; i < edges.size(); i++) {
        int a, b;
        a = edges[i].first;
        b = edges[i].second;
        if (find(nodes.begin(), nodes.end(), a) == nodes.end() &&
                find(nodes.begin(), nodes.end(), b) == nodes.end()) {
            nodes.push_back(a);
            nodes.push_back(b);
            sum += solve(edges, nodes, i+1);
            nodes.pop_back();
            nodes.pop_back();
        }
    }
    return sum;
}

int main(int argc, char *argv[]) {
    vector<pair<int, int> > edges;
    edges.push_back(make_pair<int,int>(1,2));
    edges.push_back(make_pair<int,int>(1,3));
    edges.push_back(make_pair<int,int>(2,4));
    edges.push_back(make_pair<int,int>(2,5));
    edges.push_back(make_pair<int,int>(3,5));
    edges.push_back(make_pair<int,int>(3,6));
    edges.push_back(make_pair<int,int>(4,7));
    edges.push_back(make_pair<int,int>(4,8));
    edges.push_back(make_pair<int,int>(5,8));
    edges.push_back(make_pair<int,int>(5,9));
    edges.push_back(make_pair<int,int>(6,9));
    edges.push_back(make_pair<int,int>(6,10));
    edges.push_back(make_pair<int,int>(7,11));
    edges.push_back(make_pair<int,int>(7,12));
    edges.push_back(make_pair<int,int>(8,12));
    edges.push_back(make_pair<int,int>(8,13));
    edges.push_back(make_pair<int,int>(9,13));
    edges.push_back(make_pair<int,int>(9,14));
    edges.push_back(make_pair<int,int>(10,14));
    edges.push_back(make_pair<int,int>(10,15));
    edges.push_back(make_pair<int,int>(2,3));
    edges.push_back(make_pair<int,int>(4,5));
    edges.push_back(make_pair<int,int>(5,6));
    edges.push_back(make_pair<int,int>(7,8));
    edges.push_back(make_pair<int,int>(8,9));
    edges.push_back(make_pair<int,int>(9,10));
    edges.push_back(make_pair<int,int>(11,12));
    edges.push_back(make_pair<int,int>(12,13));
    edges.push_back(make_pair<int,int>(13,14));
    edges.push_back(make_pair<int,int>(14,15));
    vector<int> empty;
    cout << solve(edges, empty) << endl;
}
