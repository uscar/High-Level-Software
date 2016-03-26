#pragma once
#include <set>

class linkable {
public:
	enum class deptype{notification,state,noteWrapper,none};
	linkable();
	deptype type;
	virtual ~linkable();
	virtual void removeDependent(linkable * dependent);
	virtual void addDependent(linkable * dependent);
	virtual void removeDependency(linkable * dependency);
	virtual void addDependency(linkable * dependency);
	virtual void manage(linkable * dependent);
	virtual void depend(linkable * dependency);

protected:
	std::set<linkable*> dependencies;
	std::set<linkable*> dependents;
};

//cpp stuff

linkable::linkable() {
	type = deptype::none;
}

linkable::~linkable() {
	for (linkable * dependent : dependents) { //delete all dependents
		dependent->removeDependency(this);
		delete dependent;
	}
	for (linkable * dependency : dependencies) { //delink all dependencies
		dependency->removeDependent(this);
	}
}

void linkable::addDependent(linkable * dependent) {
	dependents.insert(dependent);
}

void linkable::addDependency(linkable * dependency) {
	dependencies.insert(dependency);
}

void linkable::removeDependent(linkable * dependent) {
	dependents.erase(dependent);
}

void linkable::removeDependency(linkable * dependency) {
	dependencies.erase(dependency);
}

void linkable::manage(linkable * dependent) {
	addDependent(dependent);
	dependent->addDependency(this);
}

void linkable::depend(linkable * dependency) {
	addDependency(dependency);
	dependency->addDependent(this);
}
