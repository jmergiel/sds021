#pragma once

#include <functional>

/*
	A simple, generic scoped helper based on the famous Scope Guard idiom.
	Adapted for modern C++ [auto+lambda] which makes it a lot cleaner and shorter.
	Apparently there's a C++ proposal which offers something like this and probably
	is better/faster/safer/..., but I'm kinda used to using my own - at least until
	the "official" version is in widespread use...
*/

namespace scoped
{
	class OnScopeExit final
	{
		std::function<void()>	m_func;			//< function to call when this object goes out of scope
		bool					m_call = true;	//< whether to call the above function [see Dismiss()]
	public:
		explicit OnScopeExit(const std::function<void()>& foo) : m_func(std::move(foo)) {}
		~OnScopeExit()
		{
			if(m_call && m_func) m_func();
		}
		void Dismiss() noexcept { m_call = false; }	//< allows to dynamically dismiss the call
	};
}

/*
	Helper function to ease the creation of OnScopeExit objects. Typical usage:

	FILE* f = fopen(...);
	auto scopedFile = make_scoped([=]{ if(f) fclose(f); });
	// ... (use f, don't care about closing)
*/
template <typename T>
scoped::OnScopeExit make_scoped(T&& foo)
{
	using scoped::OnScopeExit;
	return OnScopeExit(std::forward<T>(foo));
}

