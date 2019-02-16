#pragma once

class Collider
{
public:
	enum Type
	{
		AABB, CIRCLE, SIZE
	};

	explicit Collider(const Type Type) : Type(Type){};
	virtual ~Collider() = default;

	Type GetType() const {return Type;}

private:
	Type Type;
};

